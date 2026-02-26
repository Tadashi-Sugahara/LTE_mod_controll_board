
#!/usr/bin/env python3
"""
List target files, then aggregate which commands precede:
 "EVENT: ERROR after max retries; GPIO20 LOW; pending restart"

Behavior (Pattern A):
 - DOES NOT print full file list
 - Prints ONLY the list of files that contain the target ERROR event
 - Prints the summary (EVENT count by preceding CMD)
 - Optionally writes CSV/Excel outputs

Assumed log examples:
 [2025/12/22,19:05:58+36] CMD: AT%SOCKETCMD="ACTIVATE",1
 [2025/12/22,19:05:58+36] EVENT: ERROR after max retries; GPIO20 LOW; pending restart

Usage:
  python log_analyzer.py

Options:
  --dir DIR              Target directory (default: downloaded_logs)
  --pattern PATTERN      Glob pattern (default: *.txt)
  --excel                Also write Excel outputs
  --csv                  Also write CSV outputs
  --limit N              Show top-N commands (default: all)
  --case-insensitive     Group commands case-insensitively for summary
  --event-ci             Match the EVENT line case-insensitively (default exact)
"""

from pathlib import Path
import re
import argparse
from collections import Counter

# Optional: pandas for file outputs (Excel/CSV)
try:
    import pandas as pd
except ImportError:
    pd = None

# --- Regex patterns（必要に応じて調整してください） ---
# CMD 行：角括弧のタイムスタンプ → "CMD:" → コマンド本文
CMD_PATTERN = re.compile(r'^\[.*?\]\s*CMD:\s*(.+)\s*$')
# イベント行：タイムスタンプ → EVENT 本文（デフォルトは完全一致）
EVENT_LITERAL = "EVENT: ERROR after max retries; GPIO20 LOW; pending restart"
EVENT_PATTERN_EXACT = re.compile(r'^\[.*?\]\s*' + re.escape(EVENT_LITERAL) + r'\s*$')
# 可読なタイムスタンプ抽出（必要なら使う）
TS_PATTERN = re.compile(r'^\[(.*?)]')

def read_lines_with_encodings(path: Path):
    """Read text lines trying utf-8 then cp932, finally utf-8 with replacement."""
    encodings = ["utf-8", "cp932"]
    for enc in encodings:
        try:
            with path.open("r", encoding=enc, errors="strict") as f:
                return f.readlines()
        except UnicodeDecodeError:
            continue
    # Fallback
    with path.open("r", encoding="utf-8", errors="replace") as f:
        return f.readlines()

def build_event_pattern(case_insensitive: bool):
    """Return compiled regex for EVENT line (exact literal vs case-insensitive)."""
    if case_insensitive:
        return re.compile(r'^\[.*?]\s*' + re.escape(EVENT_LITERAL) + r'\s*$', re.IGNORECASE)
    else:
        return EVENT_PATTERN_EXACT

def parse_log_file(path: Path, event_ci: bool):
    """
    Parse a single log file and return EVENT records linked to the preceding CMD.
    Returns a list of dicts:
    {
        'file': filename,
        'timestamp_cmd': ts_of_cmd_or_None,
        'timestamp_event': ts_of_event,
        'command': command_text_or_placeholder
    }
    """
    records = []
    lines = read_lines_with_encodings(path)
    last_cmd = None
    last_cmd_ts = None
    event_pat = build_event_pattern(event_ci)

    for line in lines:
        line_stripped = line.rstrip("\n")

        # Capture CMD line
        m_cmd = CMD_PATTERN.match(line_stripped)
        if m_cmd:
            last_cmd = m_cmd.group(1).strip()
            m_ts = TS_PATTERN.match(line_stripped)
            last_cmd_ts = m_ts.group(1).strip() if m_ts else None
            continue

        # Capture EVENT line
        if event_pat.match(line_stripped):
            m_ts = TS_PATTERN.match(line_stripped)
            evt_ts = m_ts.group(1).strip() if m_ts else None
            records.append({
                "file": path.name,
                "timestamp_cmd": last_cmd_ts,
                "timestamp_event": evt_ts,
                "command": last_cmd if last_cmd is not None else "(no preceding CMD)"
            })

    return records

def main():
    parser = argparse.ArgumentParser(description="Aggregate CMDs preceding a specific ERROR EVENT, and list only files containing that EVENT.")
    parser.add_argument("--dir", default="downloaded_logs", help="Target directory (default: downloaded_logs)")
    parser.add_argument("--pattern", default="*.txt", help="Glob pattern of log files (default: *.txt)")
    parser.add_argument("--excel", action="store_true", help="Write Excel outputs")
    parser.add_argument("--csv", action="store_true", help="Write CSV outputs")
    parser.add_argument("--limit", type=int, default=None, help="Show top-N commands")
    parser.add_argument("--case-insensitive", action="store_true",
                        help="Group commands case-insensitively for summary")
    parser.add_argument("--event-ci", action="store_true",
                        help="Match EVENT line case-insensitively (default exact)")
    args = parser.parse_args()

    target_dir = Path(args.dir)
    if not target_dir.exists():
        print(f"[ERROR] Directory not found: {target_dir}")
        return

    files = sorted(target_dir.glob(args.pattern))
    if not files:
        print(f"[INFO] No files matched: {target_dir}/{args.pattern}")
        return

    # --- (2) イベント直前CMDの集計 ---
    all_records = []
    for p in files:
        if p.is_file():
            recs = parse_log_file(p, event_ci=args.event_ci)
            all_records.extend(recs)

    if not all_records:
        print("\n[INFO] No target EVENT found in the scanned files.")
        return

    # --- (2.5) ERRORイベントを含むファイル一覧のみ表示 ---
    error_files = sorted({r["file"] for r in all_records})
    print("=== Files with target EVENT (ERROR) ===")
    for fn in error_files:
        print(fn)
    print(f"Total files (with ERROR): {len(error_files)}")

    # --- (3) 集計表示 ---
    def key_transform(cmd):
        if cmd is None:
            return "(no preceding CMD)"
        return cmd.lower() if args.case_insensitive else cmd

    counter = Counter(key_transform(r["command"]) for r in all_records)
    items = list(counter.items())
    items.sort(key=lambda x: (-x[1], x[0]))

    if args.limit is not None:
        items = items[:args.limit]

    print("\n=== EVENT count by preceding CMD ===")
    for cmd, cnt in items:
        print(f"{cnt:5d} {cmd}")
    print(f"\nTotal EVENT occurrences: {sum(counter.values())}")
    print(f"Distinct preceding CMDs: {len(counter)}")

    # --- (4) CSV/Excel 出力（必要に応じて） ---
    if args.csv or args.excel:
        if pd is None:
            print("[WARN] pandas not available; CSV/Excel outputs skipped.")
            return

        df_detail = pd.DataFrame(all_records)
        df_summary = (
            df_detail
            .assign(grouped_command=df_detail["command"].str.lower()
                    if args.case_insensitive else df_detail["command"])
            .groupby("grouped_command", as_index=False)
            .size()
            .rename(columns={"grouped_command": "command", "size": "event_count"})
            .sort_values(["event_count", "command"], ascending=[False, True])
        )

        if args.csv:
            df_detail.to_csv("event_error_gpio20_detail.csv", index=False)
            df_summary.to_csv("event_error_gpio20_summary.csv", index=False)
            print("Saved: event_error_gpio20_detail.csv, event_error_gpio20_summary.csv")

        if args.excel:
            with pd.ExcelWriter("event_error_gpio20_aggregate.xlsx", engine="openpyxl") as xw:
                df_summary.to_excel(xw, sheet_name="summary", index=False)
                df_detail.to_excel(xw, sheet_name="detail", index=False)
            print("Saved: event_error_gpio20_aggregate.xlsx (summary/detail)")

if __name__ == "__main__":
    main()
