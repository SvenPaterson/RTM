"""Plot SumpTemp over elapsed time across all sessions in a test data CSV.

The CSV may concatenate multiple test sessions, each beginning with a fresh
header row. This script loads every session, stitches them onto a single
elapsed-time axis (gaps between sessions collapsed to a nominal interval),
and annotates each session boundary as a RESTART on the chart.

It supports both CSV formats:
    * "Datetime,..."     -> first column is "MM/DD/YYYY HH:MM:SS"
    * "Elapsed(s),..."   -> first column is integer seconds since session T0
"""

from io import StringIO
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

CSV_PATH = Path(__file__).parent / "test_data.csv"

# When stitching sessions together we don't know how long the rig was off
# between sessions. Add a small fixed gap so restarts are visible without
# distorting the time axis.
RESTART_GAP = pd.Timedelta(seconds=10)


def _split_sessions(csv_path: Path) -> list[pd.DataFrame]:
    """Return one DataFrame per session, in file order."""
    with open(csv_path, "r", encoding="utf-8", errors="replace") as f:
        lines = f.readlines()

    header_key = lines[0].split(",", 1)[0].strip()
    header_idxs = [
        i for i, line in enumerate(lines)
        if line.split(",", 1)[0].strip() == header_key
    ]
    header_idxs.append(len(lines))  # sentinel for last slice

    sessions: list[pd.DataFrame] = []
    for start, end in zip(header_idxs[:-1], header_idxs[1:]):
        chunk = "".join(lines[start:end])
        df = pd.read_csv(StringIO(chunk))
        df.columns = df.columns.str.strip()
        if not df.empty:
            sessions.append(df)
    return sessions


def load_data(csv_path: Path) -> tuple[pd.DataFrame, list[float]]:
    """Load all sessions onto a single elapsed-hours axis.

    Returns (df, restart_hours) where restart_hours lists the elapsed-hour
    location of each session boundary after the first.
    """
    sessions = _split_sessions(csv_path)
    if not sessions:
        raise ValueError(f"No data sessions found in {csv_path}")

    first_cols = sessions[0].columns
    sump_col = next(c for c in first_cols if c.lower().startswith("sumptemp"))
    has_elapsed = any(c.lower().startswith("elapsed") for c in first_cols)

    pieces: list[pd.DataFrame] = []
    restart_hours: list[float] = []
    cumulative = pd.Timedelta(0)

    for i, raw in enumerate(sessions):
        if has_elapsed:
            elapsed_col = next(c for c in raw.columns if c.lower().startswith("elapsed"))
            secs = pd.to_numeric(raw[elapsed_col], errors="coerce")
            session_td = pd.to_timedelta(secs, unit="s")
        else:
            ts_col = next(c for c in raw.columns if c.lower().startswith("datetime"))
            ts = pd.to_datetime(raw[ts_col], format="%m/%d/%Y %H:%M:%S",
                                errors="coerce")
            session_td = ts - ts.iloc[0]

        sump = pd.to_numeric(raw[sump_col], errors="coerce")
        piece = pd.DataFrame({"SessionTd": session_td, "SumpTemp": sump,
                              "Session": i})
        piece = piece.dropna().reset_index(drop=True)
        if piece.empty:
            continue

        if pieces:
            cumulative += RESTART_GAP
            restart_hours.append(cumulative.total_seconds() / 3600.0)

        piece["GlobalTd"] = piece["SessionTd"] + cumulative
        cumulative += piece["SessionTd"].iloc[-1]
        pieces.append(piece)

    df = pd.concat(pieces, ignore_index=True)
    df["ElapsedHr"] = df["GlobalTd"].dt.total_seconds() / 3600.0
    return df, restart_hours


def plot_sump_temp(df: pd.DataFrame, restart_hours: list[float]) -> None:
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(df["ElapsedHr"], df["SumpTemp"], linewidth=1)

    ymin, ymax = df["SumpTemp"].min(), df["SumpTemp"].max()
    label_y = ymax + (ymax - ymin) * 0.02
    for x in restart_hours:
        ax.axvline(x, color="tab:red", linestyle="--", linewidth=0.8, alpha=0.7)
        ax.text(x, label_y, "RESTART", rotation=90, color="tab:red",
                fontsize=8, ha="right", va="top")

    ax.set_xlabel("Time from T0 (hours, restart gaps collapsed)")
    ax.set_ylabel("Sump Temperature (\u00b0C)")
    ax.set_title("Sump Temperature vs. Elapsed Time")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    data, restarts = load_data(CSV_PATH)
    print(f"Loaded {len(data)} samples across "
          f"{data['Session'].nunique()} session(s); "
          f"{len(restarts)} restart(s).")
    plot_sump_temp(data, restarts)
