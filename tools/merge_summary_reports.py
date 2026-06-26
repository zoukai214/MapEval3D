#!/usr/bin/env python3
import argparse
import csv
from datetime import datetime
import html as html_lib
from pathlib import Path
import sys
from typing import Optional
from typing import Sequence

CURRENT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CURRENT_DIR.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from mapeval3d.io_utils import SUMMARY_FIELDNAMES


REPORT_SERIES = [
    ("plane_distance_mean_p95_mean", "#1f77b4", "plane_distance_mean_p95_mean"),
    ("plane_distance_p95_threshold_mean", "#ff7f0e", "plane_distance_p95_threshold_mean"),
    ("plane_distance_thickness_p95_p5_mean", "#2ca02c", "plane_distance_thickness_p95_p5_mean"),
]


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-html", required=True)
    parser.add_argument("summary_csvs", nargs="+")
    return parser.parse_args(argv)


def read_summary_row(summary_csv: Path) -> dict:
    with summary_csv.open("r", newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"Missing header in summary CSV: {summary_csv}")
        if list(reader.fieldnames) != SUMMARY_FIELDNAMES:
            raise ValueError(f"Unexpected summary header in {summary_csv}")
        rows = list(reader)
    if len(rows) != 1:
        raise ValueError(f"Expected exactly one row in summary CSV: {summary_csv}")
    return rows[0]


def _format_number(value: str) -> str:
    try:
        return f"{float(value):.6f}"
    except ValueError:
        return html_lib.escape(value)


def _collect_chart_values(rows: Sequence[dict], fieldname: str) -> Sequence[float]:
    values = []
    for row in rows:
        values.append(float(row[fieldname]))
    return values


def _build_table_rows(rows: Sequence[dict]) -> str:
    header_cells = ["<th>id</th>"] + [f"<th>{html_lib.escape(field)}</th>" for field in SUMMARY_FIELDNAMES]
    html_rows = ["<tr>" + "".join(header_cells) + "</tr>"]
    for index, row in enumerate(rows, start=1):
        cells = [f"<td>{index}</td>"]
        for field in SUMMARY_FIELDNAMES:
            value = row[field]
            if field == "session_name":
                cells.append(f"<td>{html_lib.escape(str(value))}</td>")
            else:
                cells.append(f"<td>{_format_number(str(value))}</td>")
        html_rows.append("<tr>" + "".join(cells) + "</tr>")
    return "\n".join(html_rows)


def _scale_values(values: Sequence[float], min_value: float, max_value: float, height: float) -> Sequence[float]:
    if max_value == min_value:
        center = height / 2.0
        return [center for _ in values]
    return [height - ((value - min_value) / (max_value - min_value)) * height for value in values]


def render_svg_chart(rows: Sequence[dict]) -> str:
    if not rows:
        return "<svg viewBox='0 0 960 360' role='img' aria-label='No data'></svg>"

    chart_width = 960.0
    chart_height = 360.0
    margin_left = 70.0
    margin_right = 24.0
    margin_top = 20.0
    margin_bottom = 54.0
    plot_width = chart_width - margin_left - margin_right
    plot_height = chart_height - margin_top - margin_bottom

    series_values = []
    all_values = []
    for fieldname, _, _ in REPORT_SERIES:
        values = list(_collect_chart_values(rows, fieldname))
        series_values.append((fieldname, values))
        all_values.extend(values)

    min_value = min(all_values)
    max_value = max(all_values)
    if min_value == max_value:
        padding = 1.0 if min_value == 0.0 else abs(min_value) * 0.1
    else:
        padding = (max_value - min_value) * 0.1
    min_value -= padding
    max_value += padding

    ids = list(range(1, len(rows) + 1))
    if len(ids) == 1:
        xs = [margin_left + plot_width / 2.0]
    else:
        step = plot_width / float(len(ids) - 1)
        xs = [margin_left + step * index for index in range(len(ids))]
    ys = {
        fieldname: [
            margin_top + value
            for value in _scale_values(values, min_value, max_value, plot_height)
        ]
        for fieldname, values in series_values
    }

    svg_parts = [
        f"<svg viewBox='0 0 {int(chart_width)} {int(chart_height)}' role='img' aria-label='Summary metrics chart'>",
        f"<line x1='{margin_left}' y1='{margin_top}' x2='{margin_left}' y2='{chart_height - margin_bottom}' stroke='#333' stroke-width='1'/>",
        f"<line x1='{margin_left}' y1='{chart_height - margin_bottom}' x2='{chart_width - margin_right}' y2='{chart_height - margin_bottom}' stroke='#333' stroke-width='1'/>",
    ]

    y_tick_count = 5
    for tick_index in range(y_tick_count):
        ratio = tick_index / float(y_tick_count - 1)
        y = margin_top + plot_height * ratio
        value = max_value - (max_value - min_value) * ratio
        svg_parts.append(
            f"<line x1='{margin_left}' y1='{y:.2f}' x2='{chart_width - margin_right}' y2='{y:.2f}' stroke='#e5e7eb' stroke-width='1'/>"
        )
        svg_parts.append(
            f"<text x='{margin_left - 10}' y='{y + 4:.2f}' text-anchor='end' font-size='12' fill='#374151'>{value:.3f}</text>"
        )

    for index, frame_id in enumerate(ids):
        x = xs[index]
        svg_parts.append(
            f"<line x1='{x:.2f}' y1='{chart_height - margin_bottom}' x2='{x:.2f}' y2='{chart_height - margin_bottom + 6}' stroke='#333' stroke-width='1'/>"
        )
        svg_parts.append(
            f"<text x='{x:.2f}' y='{chart_height - margin_bottom + 22}' text-anchor='middle' font-size='12' fill='#374151'>{frame_id}</text>"
        )

    svg_parts.append(
        f"<text x='{margin_left + plot_width / 2.0:.2f}' y='{chart_height - 12.0:.2f}' text-anchor='middle' font-size='14' fill='#111827'>session_id</text>"
    )
    svg_parts.append(
        f"<text x='18' y='{margin_top + plot_height / 2.0:.2f}' text-anchor='middle' font-size='14' fill='#111827' transform='rotate(-90 18 {margin_top + plot_height / 2.0:.2f})'>m</text>"
    )

    for fieldname, color, label in REPORT_SERIES:
        points = " ".join(
            f"{xs[index]:.2f},{ys[fieldname][index]:.2f}" for index in range(len(rows))
        )
        svg_parts.append(
            f"<polyline fill='none' stroke='{color}' stroke-width='2' points='{points}'/>"
        )
        for index in range(len(rows)):
            svg_parts.append(
                f"<circle cx='{xs[index]:.2f}' cy='{ys[fieldname][index]:.2f}' r='3.5' fill='{color}'/>"
            )

    svg_parts.append("</svg>")
    return "\n".join(svg_parts)


def render_html_legend() -> str:
    items = []
    for _, color, label in REPORT_SERIES:
        items.append(
            "<div class='legend-item'>"
            f"<span class='legend-swatch' style='background:{html_lib.escape(color)};'></span>"
            f"<span>{html_lib.escape(label)}</span>"
            "</div>"
        )
    return "\n".join(items)


def render_report(rows: Sequence[dict], generated_at: str) -> str:
    table_html = _build_table_rows(rows)
    chart_svg = render_svg_chart(rows)
    legend_html = render_html_legend()
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Summary Report</title>
  <style>
    body {{
      margin: 0;
      padding: 24px;
      font-family: Arial, Helvetica, sans-serif;
      color: #111827;
      background: #f8fafc;
    }}
    .page {{
      max-width: 1400px;
      margin: 0 auto;
      display: grid;
      gap: 24px;
    }}
    .panel {{
      background: white;
      border: 1px solid #e5e7eb;
      border-radius: 12px;
      padding: 16px;
      box-shadow: 0 1px 2px rgba(0, 0, 0, 0.04);
    }}
    .watermarked {{
      position: relative;
      overflow: hidden;
    }}
    .watermarked::after {{
      content: "zoukai";
      position: absolute;
      inset: 0;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 72px;
      font-weight: 700;
      letter-spacing: 6px;
      color: rgba(17, 24, 39, 0.06);
      pointer-events: none;
      transform: rotate(-18deg);
      z-index: 2;
    }}
    .table-wrap {{
      overflow-x: auto;
      position: relative;
      z-index: 1;
    }}
    .chart-wrap {{
      position: relative;
      z-index: 1;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      font-size: 13px;
    }}
    th, td {{
      border: 1px solid #e5e7eb;
      padding: 8px 10px;
      text-align: left;
      white-space: nowrap;
    }}
    th {{
      background: #f3f4f6;
      position: sticky;
      top: 0;
      z-index: 1;
    }}
    h1 {{
      margin: 0 0 8px 0;
      font-size: 24px;
    }}
    p {{
      margin: 0;
      color: #4b5563;
    }}
    .report-meta {{
      margin-top: 4px;
      font-size: 13px;
      color: #6b7280;
    }}
    .chart-legend {{
      display: flex;
      flex-wrap: wrap;
      gap: 12px 20px;
      margin: 12px 0 16px 0;
      position: relative;
      z-index: 1;
    }}
    .legend-item {{
      display: inline-flex;
      align-items: center;
      gap: 8px;
      font-size: 13px;
      color: #111827;
      background: rgba(255, 255, 255, 0.9);
      padding: 6px 10px;
      border: 1px solid #e5e7eb;
      border-radius: 999px;
    }}
    .legend-swatch {{
      width: 18px;
      height: 3px;
      display: inline-block;
      border-radius: 999px;
    }}
    svg {{
      width: 100%;
      height: auto;
      display: block;
      position: relative;
      z-index: 1;
      background: transparent;
    }}
  </style>
</head>
<body>
  <div class="page">
    <div class="panel">
      <h1>Summary Table</h1>
      <div class="report-meta">Generated at: {html_lib.escape(generated_at)}</div>
      <p>Rows are merged in the same order as the input `summary.csv` files. The `id` column is the merge order.</p>
      <div class="table-wrap watermarked">
        <table>
          {table_html}
        </table>
      </div>
    </div>
    <div class="panel">
      <h1>Summary Metrics</h1>
      <div class="report-meta">Generated at: {html_lib.escape(generated_at)}</div>
      <p>Three mean metrics plotted against the merged row `id`.</p>
      <div class="chart-legend">
        {legend_html}
      </div>
      <div class="chart-wrap watermarked">
        {chart_svg}
      </div>
    </div>
  </div>
</body>
</html>
"""


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    summary_paths = [Path(path) for path in args.summary_csvs]
    rows = [read_summary_row(path) for path in summary_paths]
    output_html = Path(args.output_html)
    output_html.parent.mkdir(parents=True, exist_ok=True)
    generated_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    output_html.write_text(render_report(rows, generated_at), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
