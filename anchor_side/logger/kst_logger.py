"""Log CIR_BEGIN / CIR,index,hex / CIR_END from the anchor.

Install: python -m pip install pyserial
Run:     python anchor_side/logger/kst_logger.py --port COM11

CIR_BEGIN must include FP_RAW, PK_RAW, ACC, PWR, F1, F2, F3.
Raw diagnostics and decoded chip indices repeat on each sample row.
fp_index_chip is FP_RAW / 64; peak_index_chip is PK_RAW bits 30:21.
F1/F2/F3 and PWR are register values, not calibrated power in dBm.
"""

import argparse
import csv
from datetime import datetime, timedelta, timezone
from pathlib import Path
import re

COM_PORT = "COM10"
BAUD_RATE = 921600
KST = timezone(timedelta(hours=9))
DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "data"
DIAG_FIELDS = ("FP_RAW", "PK_RAW", "ACC", "PWR", "F1", "F2", "F3")
DIAG_COLUMNS = ("fp_index_raw", "peak_raw", "accum_count", "power_raw",
                "f1_raw", "f2_raw", "f3_raw", "fp_index_chip", "peak_index_chip")


def now():
    return datetime.now(KST).isoformat(timespec="milliseconds")


def decode_iq(raw_hex):
    """DW3000: little-endian I/Q, each holding a signed 18-bit value."""
    raw = bytes.fromhex(raw_hex)
    if len(raw) != 6:
        raise ValueError("Expected six bytes")

    def signed18(value):
        value &= 0x3FFFF
        return value - 0x40000 if value & 0x20000 else value

    return (signed18(int.from_bytes(raw[:3], "little")),
            signed18(int.from_bytes(raw[3:], "little")))


class CIRParser:
    def __init__(self, output):
        self.output = output
        self.writer = csv.writer(output)
        self.writer.writerow(["timestamp_kst", "packet",
                              "distance_m", "sample_index", "raw_hex", "i", "q",
                              *DIAG_COLUMNS,
                              "expected", "received", "duplicates", "invalid",
                              "missing_indices", "status"])
        self.active = None
        self.complete = 0
        self.incomplete = 0

    def finish(self, reason):
        if self.active is None:
            return
        f = self.active
        missing = sorted(set(range(f["count"])) - f["seen"])
        valid = reason == "end" and not missing and not f["duplicates"] and not f["invalid"]
        status = "complete" if valid else reason if reason != "end" else "invalid"
        # Buffer one frame so every sample carries its final quality status.
        # Preserve zero-sample frames with an empty sample row.
        for row in f["rows"] or [["", "", "", ""]]:
            self.writer.writerow([f["time"], f["packet"],
                                  f["distance"], *row, *f["diagnostics"],
                                  f["count"], len(f["seen"]),
                                  f["duplicates"], f["invalid"],
                                  " ".join(map(str, missing)), status])
        self.output.flush()
        self.complete += int(valid)
        self.incomplete += int(not valid)
        print(f"P:{f['packet']} {len(f['seen'])}/{f['count']} "
              f"{status} | complete={self.complete}, incomplete={self.incomplete}")
        self.active = None

    def feed(self, line):
        line = line.strip()
        if line.startswith("CIR_BEGIN,"):
            self.finish("new_begin_before_end")
            try:
                fields = dict(item.split(":", 1) for item in line.split(",")[1:])
                packet, count = int(fields["P"]), int(fields["N"])
                distance = float(fields["D"])
                if not 1 <= count <= 4096:
                    raise ValueError("Invalid sample count")
                diagnostics = [int(fields[key]) for key in DIAG_FIELDS]
                limits = (0xFFFF, 0xFFFFFFFF, 0xFFFF,
                          0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF)
                if any(not 0 <= value <= limit for value, limit in zip(diagnostics, limits)):
                    raise ValueError("Diagnostic value out of range")
                fp_raw, peak_raw = diagnostics[:2]
                diagnostics.extend([fp_raw / 64.0, (peak_raw >> 21) & 0x3FF])
            except (ValueError, KeyError) as exc:
                print(f"Malformed/incomplete CIR_BEGIN ({exc}); ignored. "
                      "Check firmware diagnostics and serial line truncation.")
                return
            self.active = dict(time=now(), packet=packet,
                               count=count, distance=distance,
                               diagnostics=diagnostics,
                               seen=set(), duplicates=0, invalid=0, rows=[])
        elif line.startswith("CIR,") and self.active is not None:
            f = self.active
            match = re.fullmatch(r"CIR,(\d+),([0-9a-fA-F]{12})", line)
            if match is None:
                f["invalid"] += 1
                return
            index, raw_hex = int(match[1]), match[2].upper()
            if not 0 <= index < f["count"]:
                f["invalid"] += 1
                return
            if index in f["seen"]:
                f["duplicates"] += 1
                return
            f["seen"].add(index)
            i, q = decode_iq(raw_hex)
            f["rows"].append([index, raw_hex, i, q])
        elif line.startswith("CIR_END") and self.active is not None:
            match = re.fullmatch(r"CIR_END,P:(\d+)", line)
            reason = "end" if match and int(match[1]) == self.active["packet"] else "end_mismatch"
            self.finish(reason)


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--port", default=COM_PORT)
    cli.add_argument("--baud", type=int, default=BAUD_RATE)
    cli.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = cli.parse_args()
    try:
        import serial
    except ImportError:
        cli.exit(1, "Install dependency: python -m pip install pyserial\n")

    try:
        port = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as exc:
        cli.exit(1, f"Cannot open {args.port}: {exc}\n")

    with port:
        args.output.mkdir(parents=True, exist_ok=True)
        filename = args.output / datetime.now(KST).strftime("%Y%m%d_%H%M%S_%f.csv")
        print(f"Connected: {args.port} @ {args.baud}\nSaving: {filename}\nStop: Ctrl+C")
        with filename.open("x", newline="", encoding="utf-8-sig") as output:
            parser = CIRParser(output)
            pending = bytearray()
            stop_reason = "stopped_before_end"
            try:
                while True:
                    chunk = port.read(min(max(port.in_waiting, 1), 65536))
                    if chunk:
                        pending.extend(chunk)
                        while b"\n" in pending:
                            line, _, remainder = pending.partition(b"\n")
                            pending = bytearray(remainder)
                            parser.feed(line.decode("ascii", errors="replace"))
                        if len(pending) > 65536:
                            parser.finish("oversized_line")
                            pending.clear()
            except KeyboardInterrupt:
                print("\nStopped.")
            except (serial.SerialException, OSError) as exc:
                stop_reason = "serial_error"
                print(f"Serial error: {exc}")
            finally:
                parser.finish(stop_reason)
                print(f"Saved: {filename}")


if __name__ == "__main__":
    main()
