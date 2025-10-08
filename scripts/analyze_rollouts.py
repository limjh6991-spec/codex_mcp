#!/usr/bin/env python
from __future__ import annotations
import argparse
import sys
from pathlib import Path
from src.utils.rollout_analysis import analyze_root, analyze_rollout_dir, summary_to_table, summary_to_csv, summary_to_markdown

def main():
    p = argparse.ArgumentParser(description="Analyze rollout directory")
    p.add_argument("--root", default="rollouts", help="Root rollouts directory")
    p.add_argument("--dir", default=None, help="Specific timestamped subdirectory (default=latest)")
    p.add_argument("--format", choices=["table", "csv", "md"], default="table")
    p.add_argument("--output", default=None, help="Optional output file path")
    args = p.parse_args()

    if args.dir:
        summary = analyze_rollout_dir(str(Path(args.root) / args.dir))
    else:
        summary = analyze_root(args.root)

    if args.format == "table":
        out = summary_to_table(summary)
    elif args.format == "csv":
        out = summary_to_csv(summary)
    else:
        out = summary_to_markdown(summary)

    if args.output:
        Path(args.output).write_text(out, encoding="utf-8")
    print(out)

if __name__ == "__main__":
    sys.exit(main())
