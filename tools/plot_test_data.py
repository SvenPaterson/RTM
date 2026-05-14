"""Quick-look plot for an RTM test_data.csv log.

Usage:
    python tools/plot_test_data.py [path/to/test_data.csv]

Plots SumpTemp + SealTemp vs elapsed time (in hours) and overlays loop
boundaries. Annotates the last data point so you can see how long the
test ran before it stopped (e.g. before the "sump temp not rising"
watchdog tripped).
"""
from __future__ import annotations

import sys
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt


def main(csv_path: Path) -> None:
    # File header has stray bytes for the degree symbol; read as latin-1.
    df = pd.read_csv(csv_path, encoding="latin-1")
    df.columns = [c.strip() for c in df.columns]

    # Normalize column names regardless of degree-symbol encoding.
    def col(prefix: str) -> str:
        for c in df.columns:
            if c.lower().startswith(prefix.lower()):
                return c
        raise KeyError(prefix)

    t_h = df[col("Elapsed")] / 3600.0
    sump = df[col("SumpTemp")]
    seal = df[col("SealTemp")]
    press = df[col("Press")]
    loop = df[col("Loop")]

    duration_h = float(t_h.iloc[-1])
    print(f"File:      {csv_path}")
    print(f"Samples:   {len(df)}")
    print(f"Duration:  {duration_h:.2f} h ({duration_h*60:.0f} min)")
    print(f"Loops:     {int(loop.min())} -> {int(loop.max())}")
    print(f"Sump end:  {sump.iloc[-1]:.2f} C   (min {sump.min():.2f}, max {sump.max():.2f})")

    fig, (ax1, ax2) = plt.subplots(
        2, 1, sharex=True, figsize=(12, 7), gridspec_kw={"height_ratios": [3, 1]}
    )

    ax1.plot(t_h, sump, label="Sump (C)", color="tab:red", linewidth=1.2)
    ax1.plot(t_h, seal, label="Seal (C)", color="tab:orange", linewidth=1.0, alpha=0.8)
    ax1.set_ylabel("Temp (C)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right")
    ax1.set_title(f"{csv_path.name}  -  duration {duration_h:.2f} h")

    # Mark loop boundaries.
    loop_changes = df.index[loop.diff().fillna(0) != 0]
    for idx in loop_changes:
        ax1.axvline(t_h.iloc[idx], color="gray", linestyle="--", alpha=0.4)

    # Annotate end of run.
    ax1.annotate(
        f"end @ {duration_h:.2f} h\nsump={sump.iloc[-1]:.1f}C",
        xy=(t_h.iloc[-1], sump.iloc[-1]),
        xytext=(-120, 20), textcoords="offset points",
        arrowprops=dict(arrowstyle="->", color="black"),
        fontsize=9,
    )

    ax2.plot(t_h, press, color="tab:blue", linewidth=1.0)
    ax2.set_ylabel("Press (psi)")
    ax2.set_xlabel("Elapsed (h)")
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("test_data.csv")
    if not path.is_absolute():
        path = Path(__file__).resolve().parent.parent / path
    main(path)
