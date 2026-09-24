#!/usr/bin/env python3
"""
dglib - script-based line-art diagrams for the COMPASS user manual.

Simple block diagrams and simple sequence diagrams, in the style of the time
line diagrams of make_diagrams.py: a few labeled blocks, thin arrows, plain
text notes, a statement line on top. Nothing more - no UML vocabulary (no
actors, activation bars, interaction frames, stereotypes). A diagram is a
short Python function against this library, which writes SVG and renders
PNG with Inkscape.

    from dglib import BlockDiagram, SequenceDiagram, render_png

    d = BlockDiagram("my_flow", cols=3, rows=1, title="a recording is imported")
    d.block("a", 0, 0, "Recording")
    d.block("b", 1, 0, "Import")
    d.block("c", 2, 0, "Database")
    d.arrow("a", "b", "read")
    d.arrow("b", "c", "stored")
    svg = d.write("svg")                      # svg/my_flow.svg
    render_png(svg, "../ui/import/figures/my_flow.png")

Style rules (shared with make_diagrams.py, keep every diagram consistent):
  - Canvas width 160 mm, user unit 1 mm, white background, no outer frame.
    Include the PNG with \\includegraphics[width=15cm]{figures/<name>.png}
    and without the 'frame' option - the frame is for screenshots.
    See readme_diagrams.md for the workflow and the full rules.
  - Font DejaVu Sans, which matches the Qt screenshots of the manual.
  - COMPASS palette, the three blues of the OpenATS logo: dark blue for text
    and arrows, medium blue for block borders, light blue for guide lines
    (lifelines, separators), a light tint as block fill. Dark red is used for
    problem markers only.
  - One block style. What a block is (a file, a task, a window) is said by
    its label, not by its shape or color. No color is used as the only
    carrier of information.
  - Block diagrams: data flows left to right, or top to bottom. Up to 4
    columns by 3 rows, straight or elbow arrows, a short label per arrow
    where the arrow needs one.
  - Sequence diagrams: participants across the top as blocks, time running
    downward on thin dashed lifelines, one labeled arrow per step. Passing
    time, conditions and results are plain text lines between the steps.
  - Labels are short phrases in sentence case, American English, no trailing
    period, UI strings in single quotes as in the manual text ('Pause').
    A newline in a label starts a second line.
"""

import os
import subprocess

# ----------------------------------------------------------------------------
# style
# ----------------------------------------------------------------------------

FONT = "DejaVu Sans, Verdana, sans-serif"

COL_TEXT = "#131334"        # compass_dark: text and arrows
COL_BORDER = "#3B3AA7"      # compass_medium: block borders
COL_GUIDE = "#9897EE"       # compass_light: lifelines, separators
COL_FILL = "#DCDCF7"        # block fill
COL_ALERT = "#A02020"       # problem markers only
COL_ALERT_FILL = "#F2D9D9"
COL_BG = "#FFFFFF"

FS_BLOCK = 3.2              # block labels
FS_LABEL = 2.7              # arrow labels
FS_NOTE = 2.7               # notes and the statement line

W_TOTAL = 160.0
MARGIN = 4.0
SW_BORDER = 0.4
SW_LINE = 0.35
RADIUS = 1.2
DPI = 300

_MARKERS = {COL_TEXT: "dark", COL_ALERT: "alert"}


def esc(text):
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def text_width(label, size):
    """Rough width of a DejaVu Sans label."""
    return 0.56 * size * max(len(line) for line in label.split("\n"))


def fit_size(label, size, avail, floor=2.4):
    """Font size at which the label fits into avail mm: the wanted size, or
    a smaller one down to the floor. Warns when it has to shrink, so the
    author can shorten the label instead."""
    w = text_width(label, size)
    if w <= avail:
        return size
    fitted = max(floor, size * avail / w)
    print(f"warning: label {label.split(chr(10))[0]!r} needs {w:.0f} mm, "
          f"{avail:.0f} mm available - font reduced to {fitted:.1f}")
    return fitted


def render_png(svg_path, png_path, dpi=DPI):
    """Renders an SVG to PNG with Inkscape at the manual's figure resolution."""
    os.makedirs(os.path.dirname(os.path.abspath(png_path)), exist_ok=True)
    subprocess.run(["inkscape", "--export-type=png",
                    "--export-filename=" + png_path,
                    "--export-dpi=" + str(dpi), svg_path],
                   check=True, stdout=subprocess.DEVNULL)
    return png_path


# ----------------------------------------------------------------------------
# canvas: the primitives every diagram type draws with
# ----------------------------------------------------------------------------

class Canvas:
    def __init__(self, name):
        self.name = name
        self.width = W_TOTAL
        self.height = 0.0
        self.elements = []

    def text(self, x, y, label, size=FS_LABEL, anchor="middle", color=COL_TEXT,
             bold=False):
        """Text centered vertically on y. A newline in the label starts a new
        line. Returns the estimated bounding box (x0, y0, x1, y1), which the
        diagram types use to keep lines away from text."""
        lines = label.split("\n")
        lh = size * 1.3
        y0 = y - (len(lines) - 1) * lh / 2.0 + size * 0.35
        attrs = ' font-weight="bold"' if bold else ""
        for i, line in enumerate(lines):
            self.elements.append(
                f'<text x="{x:.2f}" y="{y0 + i * lh:.2f}" font-family="{FONT}" '
                f'font-size="{size}" fill="{color}" text-anchor="{anchor}"{attrs}>'
                f'{esc(line)}</text>')
        w = text_width(label, size)
        h = len(lines) * lh
        if anchor == "start":
            bx0 = x
        elif anchor == "end":
            bx0 = x - w
        else:
            bx0 = x - w / 2.0
        return (bx0, y - h / 2.0, bx0 + w, y + h / 2.0)

    def line(self, x1, y1, x2, y2, color=COL_TEXT, width=SW_LINE, dash=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.elements.append(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{color}" stroke-width="{width}"{d} />')

    def polyline(self, points, color=COL_TEXT, width=SW_LINE, dash=None):
        """A straight or elbow connector with an arrowhead at its end."""
        d = f' stroke-dasharray="{dash}"' if dash else ""
        pts = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
        self.elements.append(
            f'<polyline points="{pts}" fill="none" stroke="{color}" '
            f'stroke-width="{width}" stroke-linejoin="round"{d} '
            f'marker-end="url(#arrow-{_MARKERS[color]})" />')

    def rect(self, x, y, w, h, fill, stroke, width=SW_BORDER, radius=RADIUS,
             dash=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.elements.append(
            f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" '
            f'rx="{radius}" ry="{radius}" fill="{fill}" stroke="{stroke}" '
            f'stroke-width="{width}"{d} />')

    def title(self, label, color=COL_TEXT):
        """Statement line above the diagram, as in the time line diagrams."""
        self.text(W_TOTAL / 2.0, 3.2, label, size=FS_NOTE, color=color)

    # -- output --------------------------------------------------------------

    def _defs(self):
        markers = []
        for color, mid in _MARKERS.items():
            markers.append(
                f'<marker id="arrow-{mid}" viewBox="0 0 10 8" refX="10" refY="4" '
                f'markerWidth="2.4" markerHeight="2.0" markerUnits="userSpaceOnUse" '
                f'orient="auto"><path d="M 0 0 L 10 4 L 0 8 z" fill="{color}" />'
                f'</marker>')
        return "<defs>\n    " + "\n    ".join(markers) + "\n  </defs>"

    def svg(self):
        self.finish()
        body = "\n  ".join(self.elements)
        return (f'<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n'
                f'<svg xmlns="http://www.w3.org/2000/svg" version="1.1" '
                f'width="{self.width}mm" height="{self.height:.2f}mm" '
                f'viewBox="0 0 {self.width} {self.height:.2f}">\n'
                f'  {self._defs()}\n'
                f'  <rect x="0" y="0" width="{self.width}" '
                f'height="{self.height:.2f}" fill="{COL_BG}" />\n'
                f'  {body}\n</svg>\n')

    def write(self, out_dir):
        """Writes <out_dir>/<name>.svg and returns its path."""
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, self.name + ".svg")
        with open(path, "w") as f:
            f.write(self.svg())
        return path

    def finish(self):
        """Hook for diagram types that draw deferred elements."""
        pass


# ----------------------------------------------------------------------------
# block diagram
# ----------------------------------------------------------------------------

class BlockDiagram(Canvas):
    """Blocks on a grid of cells, connected by straight or elbow arrows.

    cols x rows is the grid, each cell holds one block centered in it. The
    column pitch follows from the canvas width; the row pitch and the block
    size are parameters. Arrows leave and enter blocks at their edges."""

    def __init__(self, name, cols, rows, row_pitch=17.0, block_h=11.0,
                 block_w=None, top=5.0, bottom=5.0, title=None):
        super().__init__(name)
        self.col_pitch = (W_TOTAL - 2 * MARGIN) / cols
        self.block_w = block_w if block_w else self.col_pitch - 9.0
        self.block_h = block_h
        self.row_pitch = row_pitch
        self.top = top + (5.0 if title else 0.0)
        self.height = self.top + rows * row_pitch + bottom
        self.blocks = {}
        if title:
            self.title(title)

    def cx(self, col):
        return MARGIN + (col + 0.5) * self.col_pitch

    def cy(self, row):
        return self.top + (row + 0.5) * self.row_pitch

    def block(self, key, col, row, label, span=1, alert=False):
        """One block at grid cell (col, row), spanning `span` columns. alert
        marks a problem case in dark red, as the time line diagrams do."""
        x0 = self.cx(col) - self.block_w / 2.0
        x1 = self.cx(col + span - 1) + self.block_w / 2.0
        y0 = self.cy(row) - self.block_h / 2.0
        y1 = y0 + self.block_h
        if alert:
            self.rect(x0, y0, x1 - x0, y1 - y0, COL_ALERT_FILL, COL_ALERT,
                      dash="1.2,1.0")
        else:
            self.rect(x0, y0, x1 - x0, y1 - y0, COL_FILL, COL_BORDER)
        self.text((x0 + x1) / 2.0, (y0 + y1) / 2.0, label,
                  size=fit_size(label, FS_BLOCK, x1 - x0 - 2.5),
                  color=COL_ALERT if alert else COL_TEXT)
        self.blocks[key] = dict(x0=x0, y0=y0, x1=x1, y1=y1,
                                cx=(x0 + x1) / 2.0, cy=(y0 + y1) / 2.0)

    def arrow(self, src, dst, label=None, route=None, label_side=None,
              color=COL_TEXT):
        """Connects two blocks. route: 'h' (horizontal), 'v' (vertical),
        'vh' (out top/bottom, then sideways into the target), 'hv' (out
        sideways, then down/up into the target), 'hvh' (out sideways, across,
        in sideways). Chosen automatically when omitted. The label goes next
        to the longest segment it fits: above a horizontal one, right of a
        vertical one; label_side 'below' or 'left' flips that."""
        a, b = self.blocks[src], self.blocks[dst]
        right = b["cx"] > a["cx"]
        down = b["cy"] > a["cy"]
        same_row = abs(a["cy"] - b["cy"]) < 1e-6
        same_col = abs(a["cx"] - b["cx"]) < 1e-6
        if route is None:
            route = "h" if same_row else ("v" if same_col else "vh")

        if route == "h":
            xs, xe = (a["x1"], b["x0"]) if right else (a["x0"], b["x1"])
            pts = [(xs, a["cy"]), (xe, b["cy"])]
        elif route == "v":
            ys, ye = (a["y1"], b["y0"]) if down else (a["y0"], b["y1"])
            pts = [(a["cx"], ys), (b["cx"], ye)]
        elif route == "vh":
            ys = a["y1"] if down else a["y0"]
            xe = b["x0"] if right else b["x1"]
            pts = [(a["cx"], ys), (a["cx"], b["cy"]), (xe, b["cy"])]
        elif route == "hv":
            xs = a["x1"] if right else a["x0"]
            ye = b["y0"] if down else b["y1"]
            pts = [(xs, a["cy"]), (b["cx"], a["cy"]), (b["cx"], ye)]
        elif route == "hvh":
            xs, xe = (a["x1"], b["x0"]) if right else (a["x0"], b["x1"])
            xm = (xs + xe) / 2.0
            pts = [(xs, a["cy"]), (xm, a["cy"]), (xm, b["cy"]), (xe, b["cy"])]
        else:
            raise ValueError("unknown route " + route)

        self.polyline(pts, color=color)
        if label:
            self._arrow_label(pts, label, label_side, color, src, dst)

    def _arrow_label(self, pts, label, side, color, src, dst):
        def seg_len(i):
            return (abs(pts[i + 1][0] - pts[i][0]) +
                    abs(pts[i + 1][1] - pts[i][1]))

        def fits(i):
            horizontal = abs(pts[i + 1][1] - pts[i][1]) < 1e-6
            return (not horizontal or
                    seg_len(i) >= text_width(label, FS_LABEL) + 1.0)

        candidates = [i for i in range(len(pts) - 1) if fits(i)]
        if not candidates:
            print(f"warning: arrow label {label!r} fits no segment of "
                  f"{src} -> {dst} - shorten it")
            candidates = list(range(len(pts) - 1))
        best = max(candidates, key=seg_len)
        (x1, y1), (x2, y2) = pts[best], pts[best + 1]
        n = label.count("\n") + 1
        lh = FS_LABEL * 1.3
        if abs(y2 - y1) < 1e-6:                          # horizontal segment
            y = y1 + 1.4 + n * lh / 2.0 if side == "below" else y1 - 1.4 - n * lh / 2.0
            self.text((x1 + x2) / 2.0, y, label, size=FS_LABEL, color=color)
        elif side == "left":                             # vertical segment
            self.text(x1 - 1.5, (y1 + y2) / 2.0, label, size=FS_LABEL,
                      anchor="end", color=color)
        else:
            self.text(x1 + 1.5, (y1 + y2) / 2.0, label, size=FS_LABEL,
                      anchor="start", color=color)

    def note(self, col, row, label, anchor="middle", color=COL_TEXT):
        """Plain text centered on a grid cell (fractions allowed), for a
        remark next to a block."""
        self.text(self.cx(col), self.cy(row), label, size=FS_NOTE,
                  anchor=anchor, color=color)


# ----------------------------------------------------------------------------
# sequence diagram
# ----------------------------------------------------------------------------

class SequenceDiagram(Canvas):
    """Participants across the top, time running downward. Every call adds
    one row: step() draws a labeled arrow between two participants, note()
    a plain text line for passing time, a condition or a result."""

    HEAD_H = 10.0

    def __init__(self, name, participants, row=9.0, top=4.0, title=None):
        """participants: list of (key, label), left to right."""
        super().__init__(name)
        self.keys = [key for key, _ in participants]
        n = len(participants)
        self.pitch = (W_TOTAL - 2 * MARGIN) / n
        self.x = {key: MARGIN + (i + 0.5) * self.pitch
                  for i, (key, _) in enumerate(participants)}
        self.row = row
        top += 5.0 if title else 0.0
        if title:
            self.title(title)
        head_w = min(self.pitch - 8.0, 44.0)
        for key, label in participants:
            x = self.x[key]
            self.rect(x - head_w / 2.0, top, head_w, self.HEAD_H, COL_FILL,
                      COL_BORDER)
            self.text(x, top + self.HEAD_H / 2.0, label,
                      size=fit_size(label, FS_BLOCK, head_w - 2.5))
        self.y_lifeline = top + self.HEAD_H
        self.y = self.y_lifeline + row * 0.9
        self.boxes = []       # text bounding boxes, the lifelines avoid them
        self.finished = False

    def step(self, src, dst, label):
        """A labeled arrow from src to dst, one row down."""
        n = label.count("\n") + 1
        lh = FS_LABEL * 1.3
        self.y += (n - 1) * lh
        y = self.y
        xs, xe = self.x[src], self.x[dst]
        if text_width(label, FS_LABEL) > abs(xe - xs) - 2.0:
            print(f"warning: step label {label.split(chr(10))[0]!r} is wider "
                  f"than the arrow {src} -> {dst} - shorten it")
        self.polyline([(xs, y), (xe, y)])
        box = self.text((xs + xe) / 2.0,
                        y - 1.3 - FS_LABEL * 0.35 - (n - 1) * lh / 2.0,
                        label, size=FS_LABEL)
        self.boxes.append(box)
        self.y += self.row

    def note(self, label, key=None, side="right", color=COL_TEXT):
        """A plain text line: centered across the diagram, or beside the
        lifeline of key (side 'right' or 'left')."""
        n = label.count("\n") + 1
        lh = FS_NOTE * 1.3
        y = self.y - self.row * 0.35 + (n - 1) * lh / 2.0
        if key is None:
            box = self.text(W_TOTAL / 2.0, y, label, size=FS_NOTE, color=color)
        elif side == "left":
            box = self.text(self.x[key] - 2.0, y, label, size=FS_NOTE,
                            anchor="end", color=color)
        else:
            box = self.text(self.x[key] + 2.0, y, label, size=FS_NOTE,
                            anchor="start", color=color)
        self.boxes.append(box)
        self.y += self.row * 0.6 + (n - 1) * lh

    def gap(self, factor=0.5):
        """Extra vertical space, in rows."""
        self.y += self.row * factor

    def finish(self):
        """Draws the lifelines, interrupted wherever a text crosses them, so
        no label sits on a line."""
        if self.finished:
            return
        self.finished = True
        y_end = self.y - self.row * 0.4
        pad = 1.2
        for key in self.keys:
            x = self.x[key]
            cuts = sorted((y0 - pad, y1 + pad) for x0, y0, x1, y1 in self.boxes
                          if x0 - pad <= x <= x1 + pad)
            y = self.y_lifeline
            for c0, c1 in cuts:
                if c0 > y:
                    self.line(x, y, x, c0, color=COL_GUIDE, width=SW_LINE,
                              dash="1.2,1.2")
                y = max(y, c1)
            if y < y_end:
                self.line(x, y, x, y_end, color=COL_GUIDE, width=SW_LINE,
                          dash="1.2,1.2")
        self.height = y_end + 4.0
