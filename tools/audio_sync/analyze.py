#!/usr/bin/env python3
"""Compare diagnostic captures in common host-time windows.

The host timestamp only pairs nearby observations; latency itself is measured
entirely by earphone hardware. Windows must contain data from both devices.
Digital-path results do not include the codec's analog output delay.
"""

import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
import json
from pathlib import Path
import statistics


def summary(values):
    values = sorted(values)
    if not values:
        return None
    return dict(n=len(values), min=values[0], max=values[-1],
                median=statistics.median(values),
                p01=values[int((len(values)-1)*0.01)],
                p99=values[int((len(values)-1)*0.99)])


def analyze(paths, settle=10, playback_events=()):
    records = []
    events = []
    for path in paths:
        rows = [json.loads(line) for line in Path(path).read_text().splitlines()]
        records.extend(row for row in rows if "delay_us" in row)
        events.extend(row for row in rows if "event" in row)
    if not records:
        raise ValueError("No audio was captured")
    origin = min(r["time"] for r in records)
    # Only explicitly logged user/test pauses qualify for startup exclusions.
    # Never infer a restart from a gap: doing so could hide the fault under test.
    restarts = sorted(e["time"] for e in playback_events if e["event"] == "play")
    exclusions = [(origin, origin + settle)]
    paused_at = None
    for event in sorted(playback_events, key=lambda e: e["time"]):
        if event["event"] == "pause":
            paused_at = event["time"]
        elif event["event"] == "play":
            exclusions.append((paused_at or event["time"], event["time"] + settle))
            paused_at = None
    if paused_at is not None:
        exclusions.append((paused_at, max(r["time"] for r in records) + 1))
    by_side = defaultdict(list)
    windows = defaultdict(lambda: defaultdict(list))
    excluded_frames = Counter()
    for r in records:
        if any(begin <= r["time"] < end for begin, end in exclusions):
            excluded_frames[r["side"]] += 1
            continue
        r["playback_segment"] = bisect_right(restarts, r["time"])
        by_side[r["side"]].append(r)
        if r.get("timestamp_valid", True):
            windows[int(r["time"]-origin)][r["side"]].append(r["delay_us"])
    paired = [dict(second=t, left_us=statistics.median(s[0]),
                   right_us=statistics.median(s[1]),
                   right_minus_left_us=statistics.median(s[1])-statistics.median(s[0]))
              for t, s in sorted(windows.items()) if len(s[0]) >= 50 and len(s[1]) >= 50]
    sides = {}
    for side, rows in by_side.items():
        segments = defaultdict(list)
        for row in rows:
            segments[row["playback_segment"]].append(row["underruns"])
        # A recorded first-block release is one 1 ms DMA block after the
        # presentation target. Check every valid record, not only medians,
        # so brief slips cannot disappear inside a one-second comparison.
        target_errors = [r["delay_us"] - r["presentation_delay"] - 1000
                         for r in rows if r.get("timestamp_valid", True)]
        sides["left" if side == 0 else "right"] = dict(
            delay_us=summary([r["delay_us"] for r in rows if r.get("timestamp_valid", True)]),
            estimated_timestamp_frames=sum(not r.get("timestamp_valid", True) for r in rows),
            bad_frames=sum(r.get("bad_frame", False) for r in rows),
            target_error_us=summary(target_errors),
            off_target_frames_100us=sum(abs(error) > 100 for error in target_errors),
            receive_us=summary([r["receive_us"] for r in rows]),
            timestamp_estimation_us=summary([r["delay_us"]-r["estimated_delay_us"] for r in rows]),
            states=dict(Counter(str(r["states"]) for r in rows)),
            presentation_delay_us=sorted(set(r["presentation_delay"] for r in rows)),
            excluded_transition_frames=excluded_frames[side],
            new_underruns=sum(max(values)-min(values) for values in segments.values()))
    return dict(duration_s=max(r["time"] for r in records)-origin,
                settle_s=settle, sides=sides,
                excluded_intervals=exclusions,
                right_minus_left_us=summary([r["right_minus_left_us"] for r in paired]),
                paired_windows=paired,
                injections=[e for e in events if e["event"] == "inject"],
                processing_stalls=[e for e in events if e["event"] in
                                   ("processing_stall", "processing_stall_completed")],
                presentation_overrides=[e for e in events if e["event"] == "presentation_override"],
                capture_errors=[e for e in events if e["event"] not in
                                ("start", "end", "inject", "presentation_override",
                                 "processing_stall", "processing_stall_completed")])


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("captures", nargs="+")
    p.add_argument("--settle", type=float, default=10)
    p.add_argument("--output", type=Path)
    p.add_argument("--events", type=Path,
                   help="Explicit pause/play events; exclude their settling intervals")
    args = p.parse_args()
    report = analyze(args.captures, args.settle,
                     json.loads(args.events.read_text()) if args.events else ())
    if args.output:
        args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k:v for k,v in report.items() if k != "paired_windows"}, indent=2))
