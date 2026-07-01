#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path
import subprocess
import sys
from typing import Callable
from typing import List
from typing import Optional
from typing import Sequence

CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CURRENT_DIR.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


DEFAULT_CSV_PATH = PROJECT_ROOT / "config" / "intersection_30.csv"
DEFAULT_DATASET_ROOT = Path("/mnt/oss/gacrnd-annotation/ruqi/upm/datasets")
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "output" / "session_ouput"
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config" / "local_block_eval.json"
DEFAULT_REPORT_PATH = PROJECT_ROOT / "output" / "session_ouput" / "batch_summary_report.html"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default=str(DEFAULT_CSV_PATH))
    parser.add_argument("--dataset-root", default=str(DEFAULT_DATASET_ROOT))
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    parser.add_argument("--report-html", default=str(DEFAULT_REPORT_PATH))
    return parser.parse_args(argv)


def read_session_names(csv_path: Path) -> List[str]:
    session_names = []
    with csv_path.open("r", newline="") as handle:
        reader = csv.reader(handle)
        for row in reader:
            for cell in row:
                session_name = cell.strip()
                if session_name:
                    session_names.append(session_name)
    return session_names


def build_extract_command(
    input_root: Path,
    session_path: Path,
    output_dir: Path,
    config_path: Path,
) -> List[str]:
    return [
        sys.executable,
        str(PROJECT_ROOT / "tools" / "extract_keyframe_local_blocks.py"),
        "--input-root",
        str(input_root),
        "--session-path",
        str(session_path),
        "--output-dir",
        str(output_dir),
        "--config",
        str(config_path),
    ]


def build_merge_command(summary_paths: Sequence[Path], report_html: Path) -> List[str]:
    return [
        sys.executable,
        str(PROJECT_ROOT / "tools" / "merge_summary_reports.py"),
        "--output-html",
        str(report_html),
        *[str(path) for path in summary_paths],
    ]


def run_batch(
    csv_path: Path,
    dataset_root: Path,
    output_root: Path,
    config_path: Path,
    runner: Callable[[Sequence[str]], int],
    report_html: Optional[Path] = None,
) -> int:
    session_names = read_session_names(csv_path)
    summary_paths = []
    failed_count = 0
    skipped_count = 0
    ok_count = 0

    for session_name in session_names:
        session_path = dataset_root / session_name
        label_dir = session_path / "prelabel" / "RoadMark_label"
        laz_dir = label_dir / "laz"
        input_gps_msg = label_dir / "gps_msg.txt"
        output_dir = output_root / session_name

        laz_files = sorted(laz_dir.glob("*.laz")) if laz_dir.exists() else []
        if len(laz_files) == 0:
            print(
                f"[SKIP] {session_name}: expected at least 1 laz in {laz_dir}, found 0",
                flush=True,
            )
            skipped_count += 1
            continue
        if not input_gps_msg.is_file():
            print(f"[SKIP] {session_name}: missing gps_msg.txt at {input_gps_msg}", flush=True)
            skipped_count += 1
            continue

        if len(laz_files) > 1:
            print(
                f"[MERGE] {session_name}: evaluating {len(laz_files)} laz files in memory",
                flush=True,
            )
        command = build_extract_command(
            input_root=session_path,
            session_path=session_path,
            output_dir=output_dir,
            config_path=config_path,
        )
        print(f"[RUN] {session_name}", flush=True)
        exit_code = runner(command)
        if exit_code != 0:
            print(f"[FAIL] {session_name}: exit code {exit_code}", flush=True)
            failed_count += 1
            continue

        summary_path = output_dir / "summary.csv"
        if summary_path.is_file():
            summary_paths.append(summary_path)
            ok_count += 1
            print(f"[OK] {session_name}: {summary_path}", flush=True)
        else:
            print(f"[FAIL] {session_name}: missing generated summary.csv", flush=True)
            failed_count += 1

    if report_html is not None and summary_paths:
        report_html.parent.mkdir(parents=True, exist_ok=True)
        merge_command = build_merge_command(summary_paths, report_html)
        print(f"[MERGE] {len(summary_paths)} summaries -> {report_html}", flush=True)
        merge_exit_code = runner(merge_command)
        if merge_exit_code != 0:
            print(f"[FAIL] merge_summary_reports.py: exit code {merge_exit_code}", flush=True)
            failed_count += 1

    print(
        f"[DONE] ok={ok_count} skipped={skipped_count} failed={failed_count}",
        flush=True,
    )
    return 1 if failed_count else 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    return run_batch(
        csv_path=Path(args.csv),
        dataset_root=Path(args.dataset_root),
        output_root=Path(args.output_root),
        config_path=Path(args.config),
        report_html=Path(args.report_html),
        runner=lambda command: subprocess.run(command).returncode,
    )


if __name__ == "__main__":
    raise SystemExit(main())
