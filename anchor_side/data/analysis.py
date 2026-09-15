import argparse
import re
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib

parser = argparse.ArgumentParser(description="Plot raw Final CIR sessions")
parser.add_argument("--file", "--folder", dest="folder", type=Path,
                    help="CSV file (or legacy session folder) beside this script, or absolute path")
parser.add_argument("--packet", type=int, help="Packet to plot; default is first valid packet")
parser.add_argument("--no-show", action="store_true")
args = parser.parse_args()
if args.no_show:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

base = Path(__file__).resolve().parent
diag_columns = ["fp_index_raw", "peak_raw", "accum_count", "power_raw",
                "f1_raw", "f2_raw", "f3_raw", "fp_index_chip", "peak_index_chip"]
if args.folder is None:
    sessions = sorted((p for p in base.iterdir()
                      if (p.is_file() and re.search(r"\d{8}_\d{6}_\d{6}\.csv$", p.name))
                      or (p.is_dir() and (p / "frames.csv").is_file()
                          and (p / "samples.csv").is_file())),
                      key=lambda p: (re.search(r"\d{8}_\d{6}_\d{6}$", p.stem).group()
                                     if re.search(r"\d{8}_\d{6}_\d{6}$", p.stem) else p.name))
    if not sessions:
        parser.error(f"No CIR sessions found in {base}")
    folder = sessions[-1]
else:
    folder = args.folder if args.folder.is_absolute() else base / args.folder

if folder.is_file():
    samples = pd.read_csv(folder, dtype={"raw_hex": str, "missing_indices": str})
    frame_columns = ["timestamp_kst", "packet", "distance_m",
                     "expected", "received", "duplicates", "invalid", "missing_indices", "status"]
    frame_columns += [c for c in diag_columns if c in samples.columns]
    frames = samples[frame_columns].drop_duplicates("packet")
else:
    frames = pd.read_csv(folder / "frames.csv")
    samples = pd.read_csv(folder / "samples.csv", dtype={"raw_hex": str})

valid = frames[
    (frames["status"] == "complete")
    & (frames["expected"] == 1016)
    & (frames["received"] == 1016)
    & (frames["duplicates"] == 0)
    & (frames["invalid"] == 0)
]

if valid.empty:
    raise RuntimeError("정상 수집된 프레임이 없습니다.")

# Verify sample rows as well as the frame completion marker.
good_ids = []
for fid, group in samples[samples["packet"].isin(valid["packet"])].groupby("packet"):
    if (len(group) == 1016 and set(group["sample_index"]) == set(range(1016))
            and all(group[c].nunique(dropna=False) == 1
                    for c in ["timestamp_kst", "distance_m", *diag_columns] if c in group)
            and np.isfinite(group[["i", "q"]].to_numpy(dtype=float)).all()):
        good_ids.append(fid)
valid = valid[valid["packet"].isin(good_ids)].sort_values("packet").copy()
if valid.empty:
    parser.error("No frames passed sample validation")
packet = args.packet if args.packet is not None else int(valid.iloc[0]["packet"])
if packet not in good_ids:
    parser.error(f"Frame {packet} is not complete")
output = (folder.parent / "analysis" / folder.stem) if folder.is_file() else folder / "analysis"
output.mkdir(parents=True, exist_ok=True)
cir = samples[samples["packet"] == packet].sort_values("sample_index")

i = cir["i"].to_numpy(dtype=float)
q = cir["q"].to_numpy(dtype=float)
amplitude = np.hypot(i, q)

plt.figure(figsize=(12, 4))
plt.plot(cir["sample_index"], amplitude)
plt.axvline(int(np.argmax(amplitude)), color="black", linestyle=":", label="I/Q maximum")
for col, label, color in [("fp_index_chip", "Chip first path", "tab:red"),
                           ("peak_index_chip", "Chip peak", "tab:green")]:
    if col in cir and pd.notna(cir.iloc[0][col]):
        plt.axvline(cir.iloc[0][col], color=color, linestyle="--",
                    label=f"{label}: {cir.iloc[0][col]:.3f}")
plt.legend()
plt.xlabel("CIR sample index")
plt.ylabel("Amplitude (raw units)")
plt.title(f"Final CIR — packet {int(packet)}")
plt.grid(alpha=0.3)
plt.tight_layout()
plt.savefig(output / f"cir_packet_{packet}.png", dpi=160)
# Also save a detailed view around the main arrival.
plt.xlim(max(0, int(np.argmax(amplitude)) - 30), min(1015, int(np.argmax(amplitude)) + 70))
plt.savefig(output / f"cir_packet_{packet}_zoom.png", dpi=160)

selected = samples[samples["packet"].isin(good_ids)].copy()
selected["amplitude"] = np.hypot(selected["i"].to_numpy(dtype=float),
                                  selected["q"].to_numpy(dtype=float))
matrix = selected.pivot(index="packet", columns="sample_index", values="amplitude")
matrix = matrix.reindex(index=valid["packet"], columns=range(1016))
values = matrix.to_numpy()
metrics = valid[["timestamp_kst", "packet", "distance_m",
                 *[c for c in diag_columns if c in valid.columns]]].copy()
metrics["peak_index_iq"] = values.argmax(axis=1)
metrics["peak_amplitude"] = values.max(axis=1)
metrics["sum_squared_amplitude"] = np.square(values).sum(axis=1)
if "fp_index_chip" in metrics:
    metrics["iq_peak_minus_fp_samples"] = metrics["peak_index_iq"] - metrics["fp_index_chip"]
if "peak_index_chip" in metrics:
    metrics["iq_minus_chip_peak_samples"] = metrics["peak_index_iq"] - metrics["peak_index_chip"]
    if "fp_index_chip" in metrics:
        metrics["chip_peak_minus_fp_samples"] = metrics["peak_index_chip"] - metrics["fp_index_chip"]
if "accum_count" in metrics:
    accum = metrics["accum_count"].where(metrics["accum_count"] > 0)
    metrics["peak_amplitude_per_accum"] = metrics["peak_amplitude"] / accum
metrics.to_csv(output / "frame_metrics.csv", index=False, encoding="utf-8-sig")

# Relative amplitude per frame, not calibrated power or SNR.
peaks = np.maximum(values.max(axis=1, keepdims=True), np.finfo(float).tiny)
relative_db = 20 * np.log10(np.maximum(values / peaks, 1e-3))
fig, ax = plt.subplots(figsize=(12, 6), layout="constrained")
im = ax.imshow(relative_db, aspect="auto", origin="lower", interpolation="nearest",
               vmin=-60, vmax=0, cmap="viridis")
ax.set(xlabel="CIR sample index", ylabel="Complete-frame order (0-based)",
       title="Final CIR | normalized separately for each frame")
fig.colorbar(im, ax=ax, label="Amplitude relative to frame maximum (dB)")
fig.savefig(output / "cir_heatmap.png", dpi=160)

times = pd.to_datetime(metrics["timestamp_kst"], utc=True)
elapsed = (times - times.iloc[0]).dt.total_seconds()
fig, ax = plt.subplots(figsize=(12, 4), layout="constrained")
ax.plot(elapsed, metrics["distance_m"], ".-", linewidth=0.8)
ax.set(xlabel="Elapsed PC receipt time (s)", ylabel="DS-TWR distance (m)",
       title="Distance of complete frames (not ground-truth error)")
ax.grid(alpha=0.3)
fig.savefig(output / "distance.png", dpi=160)

if "fp_index_chip" in metrics or "peak_index_chip" in metrics:
    fig, ax = plt.subplots(figsize=(12, 4), layout="constrained")
    for col, label in [("fp_index_chip", "Chip first path"),
                       ("peak_index_chip", "Chip peak"), ("peak_index_iq", "I/Q maximum")]:
        if col in metrics:
            ax.plot(elapsed, metrics[col], ".-", linewidth=0.8, label=label)
    ax.set(xlabel="Elapsed PC receipt time (s)", ylabel="CIR sample index",
           title="Chip diagnostics compared with raw I/Q")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.savefig(output / "path_indices.png", dpi=160)

summary = (f"Session: {folder.name}\nRecorded frames: {len(frames)}\n"
           f"Validated complete frames: {len(valid)} ({len(valid)/len(frames):.1%})\n"
           f"Excluded frames: {len(frames)-len(valid)}\n"
           "Completeness excludes Poll attempts that produced no recorded frame.\n"
           "Peak index is not necessarily the first path or physical distance.\n"
           "Heatmap uses per-frame normalization; metrics use raw amplitudes.\n")
summary += "Available chip diagnostics: " + ", ".join(c for c in diag_columns if c in metrics) + "\n"
summary += "F1/F2/F3 and power_raw retain register units; not dBm.\n"
(output / "summary.txt").write_text(summary, encoding="utf-8")
print(summary)
print(f"Saved: {output}")
if not args.no_show:
    plt.show()
plt.close("all")
