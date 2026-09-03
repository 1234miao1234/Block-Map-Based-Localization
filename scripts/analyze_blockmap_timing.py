#!/usr/bin/env python3
"""Summarize Block-Map steady-state matching timings for batch CSV output."""

from __future__ import annotations

import argparse
import math
import re
import statistics
from pathlib import Path


FRAME_MARKER = "[BLOCKMAP_TIMING]"


def parse_record(line: str) -> dict[str, str] | None:
    marker_pos = line.find(FRAME_MARKER)
    if marker_pos < 0:
        return None
    record: dict[str, str] = {}
    for field in line[marker_pos + len(FRAME_MARKER):].strip().split(","):
        key, separator, value = field.strip().partition("=")
        if separator:
            record[key] = value
    return record


def integer(record: dict[str, str], key: str) -> int:
    # roslaunch output may append an ANSI colour reset after the last field.
    match = re.match(r"[+-]?\d+", record.get(key, "0"))
    return int(match.group(0)) if match else 0


def metric_stats(values: list[int]) -> tuple[float, float, int]:
    if not values:
        return 0.0, 0.0, 0
    ordered = sorted(values)
    p95_index = max(0, math.ceil(0.95 * len(ordered)) - 1)
    return statistics.fmean(ordered), statistics.median(ordered), ordered[p95_index]


def csv_header() -> str:
    return ",".join((
        "correspondence_mean_us",
        "correspondence_median_us",
        "correspondence_p95_us",
        "matcher_total_mean_us",
        "correspondence_sum_s",
        "matcher_total_sum_s",
    ))


def selected_records(records: list[dict[str, str]]) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    successful_all = [record for record in records if integer(record, "success") == 1]
    successful_steady = [
        record for record in successful_all if integer(record, "steady") == 1
    ]
    return successful_steady, successful_all


def csv_row(steady: list[dict[str, str]], successful_all: list[dict[str, str]]) -> str:
    correspondence = [integer(record, "correspondence_us") for record in steady]
    matcher = [integer(record, "matcher_total_us") for record in steady]
    correspondence_mean, correspondence_median, correspondence_p95 = metric_stats(correspondence)
    matcher_mean, _, _ = metric_stats(matcher)
    return ",".join((
        f"{correspondence_mean:.3f}",
        f"{correspondence_median:.3f}",
        f"{correspondence_p95:.3f}",
        f"{matcher_mean:.3f}",
        f"{sum(integer(record, 'correspondence_us') for record in successful_all) / 1_000_000.0:.6f}",
        f"{sum(integer(record, 'matcher_total_us') for record in successful_all) / 1_000_000.0:.6f}",
    ))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=Path)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--csv-header", action="store_true")
    group.add_argument("--csv-row", action="store_true")
    args = parser.parse_args()

    if args.csv_header:
        print(csv_header())
        return 0
    if args.log is None:
        parser.error("log is required unless --csv-header is used")

    records = []
    with args.log.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            record = parse_record(line)
            if record is not None:
                records.append(record)
    if not records:
        parser.error(f"no {FRAME_MARKER} records in {args.log}")

    steady, successful_all = selected_records(records)
    if not steady:
        parser.error(f"no successful steady-state {FRAME_MARKER} records in {args.log}")
    if args.csv_row:
        print(csv_row(steady, successful_all))
        return 0

    correspondence = [integer(record, "correspondence_us") for record in steady]
    matcher = [integer(record, "matcher_total_us") for record in steady]
    c_mean, c_median, c_p95 = metric_stats(correspondence)
    m_mean, m_median, m_p95 = metric_stats(matcher)
    print("Block-Map selected steady-state metrics")
    print(f"  frames: {len(steady)}")
    print(f"  correspondence_us: mean={c_mean:.2f}, median={c_median:.2f}, p95={c_p95}")
    print(f"  matcher_total_us: mean={m_mean:.2f}, median={m_median:.2f}, p95={m_p95}")
    print("Block-Map successful full-run accumulated time")
    print(f"  correspondence_sum_s: {sum(integer(r, 'correspondence_us') for r in successful_all) / 1_000_000.0:.6f}")
    print(f"  matcher_total_sum_s: {sum(integer(r, 'matcher_total_us') for r in successful_all) / 1_000_000.0:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
