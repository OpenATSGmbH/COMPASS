#!/usr/bin/env python3
"""
Generates the user manual line-art diagrams as SVG and renders them to PNG.

Usage:
    python3 make_diagrams.py            # write all SVGs and render all PNGs
    python3 make_diagrams.py --svg-only # write the SVGs only

The SVG sources are written to diagrams/svg/. The PNGs are written into the
figures/ folder of the chapter that includes them. Both are checked in, so the
manual can be built without this script.

Style rules (keep any new diagram consistent with them):
  - Canvas width 160 mm, user unit 1 mm, white background, no outer frame.
  - Font DejaVu Sans, which matches the Qt screenshots of the manual.
  - COMPASS palette - dark blue for text and axis, medium blue for borders,
    light blue for fills. Dark red is used for problem markers only.
  - Time runs from left to right on a single arrow axis. Recording files are
    blocks above the axis, one row per file that needs its own row.
  - No color is used as the only carrier of information. Every block and every
    marker also has a text label.
"""

import argparse
import os
import subprocess

# ----------------------------------------------------------------------------
# style
# ----------------------------------------------------------------------------

FONT = "DejaVu Sans, Verdana, sans-serif"

COL_TEXT = "#131334"   # compass_dark
COL_AXIS = "#131334"
COL_DAY = "#9897EE"    # compass_light, day boundary lines
COL_BORDER = "#3B3AA7" # compass_medium, block borders
COL_FILL = "#DCDCF7"   # block fill
COL_ALERT = "#A02020"  # time jump marker, removed data
COL_ALERT_FILL = "#F2D9D9"

FS_BLOCK = 3.2         # block label
FS_LABEL = 2.7         # tick and time labels
FS_NOTE = 2.7          # notes

W_TOTAL = 160.0
MARGIN_L = 4.0
MARGIN_R = 14.0
ROW_H = 9.0
ROW_GAP = 5.0          # room for the time labels above a block
SEG_GAP = 7.0          # gap of a broken axis

OUT_SVG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "svg")
OUT_PNG = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "..", "ui", "import", "figures"))
DPI = 300


def esc(text):
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


class Diagram:
    """One time line diagram. Times are given in hours, 0.0 is Day 1 00:00."""

    def __init__(self, name, segments, num_rows, top_pad=8.0, axis_gap=3.0):
        # segments: list of (t_start, t_end) mapped left to right, split by a
        # break marker. One segment is the normal case, two segments are used
        # when the interesting parts are minutes apart inside a 24 h recording.
        self.name = name
        self.segments = segments
        self.num_rows = num_rows
        self.top_pad = top_pad
        self.elements = []

        avail = W_TOTAL - MARGIN_L - MARGIN_R - SEG_GAP * (len(segments) - 1)
        self.seg_width = avail / len(segments)

        self.axis_y = top_pad + num_rows * (ROW_H + ROW_GAP) + axis_gap
        self.height = self.axis_y + 11.0

    # -- coordinates ---------------------------------------------------------

    def x(self, t):
        for i, (t0, t1) in enumerate(self.segments):
            if t0 <= t <= t1 or i == len(self.segments) - 1:
                x0 = MARGIN_L + i * (self.seg_width + SEG_GAP)
                return x0 + (t - t0) / (t1 - t0) * self.seg_width
        raise ValueError("time outside of the diagram")

    def row_y(self, row):
        return self.top_pad + row * (ROW_H + ROW_GAP)

    # -- elements ------------------------------------------------------------

    def text(self, x, y, label, size=FS_LABEL, anchor="middle", color=COL_TEXT,
             bold=False):
        weight = ' font-weight="bold"' if bold else ""
        self.elements.append(
            f'<text x="{x:.2f}" y="{y:.2f}" font-family="{FONT}" '
            f'font-size="{size}" fill="{color}" text-anchor="{anchor}"{weight}>'
            f'{esc(label)}</text>')

    def line(self, x1, y1, x2, y2, color=COL_AXIS, width=0.3, dash=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.elements.append(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{color}" stroke-width="{width}"{d} />')

    def rect(self, x, y, w, h, fill, stroke, width=0.4, radius=1.2, dash=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.elements.append(
            f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" '
            f'rx="{radius}" ry="{radius}" fill="{fill}" stroke="{stroke}" '
            f'stroke-width="{width}"{d} />')

    # -- building blocks -----------------------------------------------------

    def axis(self):
        y = self.axis_y
        for i in range(len(self.segments)):
            x0 = MARGIN_L + i * (self.seg_width + SEG_GAP)
            x1 = x0 + self.seg_width
            if i == len(self.segments) - 1:
                x1 += 8.0
            self.line(x0 - 2.0, y, x1, y, width=0.4)
            if i == len(self.segments) - 1:
                self.elements.append(
                    f'<path d="M {x1:.2f} {y:.2f} l -2.2 -1.1 l 0 2.2 z" '
                    f'fill="{COL_AXIS}" />')
                self.text(x1 + 3.0, y + 1.0, "t", size=FS_LABEL)
            else:
                # break marker between two segments
                xb = x1 + SEG_GAP / 2.0
                for off in (-0.9, 0.9):
                    self.elements.append(
                        f'<path d="M {xb + off - 1.0:.2f} {y + 1.6:.2f} '
                        f'l 1.0 -1.6 l 1.0 1.6" fill="none" '
                        f'stroke="{COL_AXIS}" stroke-width="0.4" />')

    def day_mark(self, t, day_label, time_label="00:00"):
        x = self.x(t)
        self.line(x, self.top_pad - 2.0, x, self.axis_y, color=COL_DAY,
                  width=0.35, dash="1.2,1.2")
        self.line(x, self.axis_y, x, self.axis_y + 1.6, width=0.4)
        self.text(x, self.axis_y + 4.6, day_label, size=FS_LABEL, bold=True)
        self.text(x, self.axis_y + 8.0, time_label, size=FS_LABEL)

    def tick(self, t, day_label, time_label, anchor="middle"):
        x = self.x(t)
        x_text = x
        if anchor == "start":
            x_text = x - 1.0
        elif anchor == "end":
            x_text = x + 1.0
        self.line(x, self.axis_y, x, self.axis_y + 1.2, width=0.35)
        if day_label:
            self.text(x_text, self.axis_y + 4.6, day_label, size=FS_LABEL,
                      anchor=anchor)
            self.text(x_text, self.axis_y + 8.0, time_label, size=FS_LABEL,
                      anchor=anchor)
        else:
            self.text(x_text, self.axis_y + 4.6, time_label, size=FS_LABEL,
                      anchor=anchor)

    def block(self, row, t0, t1, label, t0_label=None, t1_label=None):
        y = self.row_y(row)
        x0, x1 = self.x(t0), self.x(t1)
        self.rect(x0, y, x1 - x0, ROW_H, COL_FILL, COL_BORDER)
        self.text((x0 + x1) / 2.0, y + ROW_H / 2.0 + 1.1, label, size=FS_BLOCK)
        if t0_label:
            self.text(x0 + 0.5, y - 1.2, t0_label, size=FS_LABEL, anchor="start")
        if t1_label:
            self.text(x1 - 0.5, y - 1.2, t1_label, size=FS_LABEL, anchor="end")

    def removed(self, row, t0, t1, label=None):
        """Marks a part of a block as removed by the Time of Day filter."""
        y = self.row_y(row)
        x0, x1 = self.x(t0), self.x(t1)
        self.rect(x0, y, x1 - x0, ROW_H, COL_ALERT_FILL, COL_ALERT,
                  width=0.4, dash="1.2,1.0")
        if label:
            self.text((x0 + x1) / 2.0, y + ROW_H + 3.4, label, size=FS_NOTE,
                      color=COL_ALERT)

    def jump_marker(self, t, row, label):
        """Marks the point at which the 24h time jump is detected."""
        x = self.x(t)
        y = self.row_y(row)
        self.line(x, y - 0.8, x, y + ROW_H + 0.8, color=COL_ALERT, width=0.6)
        self.elements.append(
            f'<path d="M {x:.2f} {y - 1.0:.2f} l -1.3 -2.0 l 2.6 0 z" '
            f'fill="{COL_ALERT}" />')
        self.text(x, y - 3.8, label, size=FS_NOTE, color=COL_ALERT)

    def note(self, x, y, label, anchor="middle", color=COL_TEXT):
        self.text(x, y, label, size=FS_NOTE, anchor=anchor, color=color)

    def title(self, label, color=COL_TEXT):
        """Result statement above the first row."""
        self.text((MARGIN_L + W_TOTAL - MARGIN_R) / 2.0, 3.2, label,
                  size=FS_NOTE, color=color)

    def brace_note(self, t0, t1, row, label):
        """Horizontal extent marker below a row, used for the filter window."""
        y = self.row_y(row) + ROW_H + 2.4
        x0, x1 = self.x(t0), self.x(t1)
        self.line(x0, y, x1, y, width=0.35)
        self.line(x0, y - 1.0, x0, y + 1.0, width=0.35)
        self.line(x1, y - 1.0, x1, y + 1.0, width=0.35)
        self.text((x0 + x1) / 2.0, y + 3.6, label, size=FS_NOTE)

    # -- output --------------------------------------------------------------

    def svg(self):
        body = "\n  ".join(self.elements)
        return (f'<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n'
                f'<svg xmlns="http://www.w3.org/2000/svg" version="1.1" '
                f'width="{W_TOTAL}mm" height="{self.height:.2f}mm" '
                f'viewBox="0 0 {W_TOTAL} {self.height:.2f}">\n'
                f'  <rect x="0" y="0" width="{W_TOTAL}" '
                f'height="{self.height:.2f}" fill="#FFFFFF" />\n'
                f'  {body}\n</svg>\n')

    def write(self):
        path = os.path.join(OUT_SVG, self.name + ".svg")
        with open(path, "w") as f:
            f.write(self.svg())
        return path


# ----------------------------------------------------------------------------
# diagrams
# ----------------------------------------------------------------------------

def case_a():
    """One file inside one day."""
    d = Diagram("asterix_import_ts_a", [(0.0, 24.0)], 1)
    d.day_mark(0.0, "Day 1")
    d.day_mark(24.0, "Day 2")
    d.block(0, 6.0, 18.0, "File 1 (Radar)", "06:00", "18:00")
    d.title("all Times of Day map to Day 1")
    d.axis()
    return d


def case_b():
    """One file across midnight."""
    d = Diagram("asterix_import_ts_b", [(18.0, 30.0)], 1, top_pad=11.0)
    d.day_mark(24.0, "Day 2")
    d.tick(18.0, "Day 1", "18:00", anchor="start")
    d.tick(30.0, "Day 2", "06:00", anchor="end")
    d.block(0, 22.0, 26.0, "File 1 (Radar)", "22:00", "02:00")
    d.jump_marker(24.0, 0, "24h time jump detected, date set to Day 2")
    d.axis()
    return d


def case_c():
    """Two files of the same day, each starting again at its own time."""
    d = Diagram("asterix_import_ts_c", [(0.0, 24.0)], 2)
    d.day_mark(0.0, "Day 1")
    d.day_mark(24.0, "Day 2")
    d.block(0, 0.02, 23.98, "File 1 (SMR)", "00:01", "23:59")
    d.block(1, 0.02, 23.98, "File 2 (MLAT)", "00:01", "23:59")
    d.title("both files start again at 00:00, both belong to Day 1")
    d.axis()
    return d


def case_d():
    """Two files that continue one time line, the day changes at the cut."""
    d = Diagram("asterix_import_ts_d", [(18.0, 30.0)], 1, top_pad=11.0)
    d.day_mark(24.0, "Day 2")
    d.tick(18.0, "Day 1", "18:00", anchor="start")
    d.tick(30.0, "Day 2", "06:00", anchor="end")
    d.block(0, 20.0, 24.0, "File 1", "20:00", None)
    d.block(0, 24.0, 28.0, "File 2", None, "04:00")
    d.jump_marker(24.0, 0, "24h time jump detected at the start of File 2")
    d.axis()
    return d


def case_e():
    """One 24 h file that also holds minutes of the day before and after."""
    # broken axis, the minutes around both midnights are the point of interest
    d = Diagram("asterix_import_ts_e",
                [(-0.1667, 0.4167), (23.5833, 24.1667)], 1, axis_gap=10.0)
    d.day_mark(0.0, "Day 1")
    d.day_mark(24.0, "Day 2")
    d.tick(-0.1667, None, "23:50", anchor="start")
    d.tick(24.1667, None, "00:10", anchor="end")
    d.block(0, -0.0333, 24.0333, "File 1 (24 h recording)", "23:58", "00:02")
    d.removed(0, -0.0333, 0.0833)
    d.removed(0, 23.9167, 24.0333)
    # beside the day boundary line, so the dashed line does not cross the text
    d.note(d.x(0.0) - 2.0, d.row_y(0) + ROW_H + 4.0, "filtered out",
           anchor="end", color=COL_ALERT)
    d.note(d.x(24.0) + 2.0, d.row_y(0) + ROW_H + 4.0, "filtered out",
           anchor="start", color=COL_ALERT)
    d.title("no time jump detection, all data gets Day 1")
    d.note((MARGIN_L + W_TOTAL - MARGIN_R) / 2.0, d.axis_y + 8.0,
           "(axis not to scale)")
    d.axis()
    return d


def case_f():
    """Several files of one day, with data close to both midnights."""
    d = Diagram("asterix_import_ts_f", [(0.0, 24.0)], 3)
    d.day_mark(0.0, "Day 1")
    d.day_mark(24.0, "Day 2")
    d.block(0, 0.0, 23.99, "File 1 (SMR)", "00:00", "23:59")
    d.block(1, 0.0, 12.0, "File 2 (MLAT)", "00:00", "12:00")
    d.block(2, 12.0, 23.99, "File 3 (ADS-B)", "12:00", "23:59")
    d.title("every file belongs to Day 1, Times of Day near midnight "
            "must not change the date")
    d.axis()
    return d


DIAGRAMS = [case_a, case_b, case_c, case_d, case_e, case_f]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--svg-only", action="store_true")
    args = parser.parse_args()

    os.makedirs(OUT_SVG, exist_ok=True)

    for builder in DIAGRAMS:
        diagram = builder()
        svg_path = diagram.write()
        print("wrote " + svg_path)

        if args.svg_only:
            continue

        png_path = os.path.join(OUT_PNG, diagram.name + ".png")
        subprocess.run(["inkscape", "--export-type=png",
                        "--export-filename=" + png_path,
                        "--export-dpi=" + str(DPI), svg_path],
                       check=True, stdout=subprocess.DEVNULL)
        print("rendered " + png_path)


if __name__ == "__main__":
    main()
