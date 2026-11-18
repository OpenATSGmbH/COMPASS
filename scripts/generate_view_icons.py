#!/usr/bin/env python3
"""
Generates the View Deck icons (one per view type) in the style of the Flight Deck icons:
black line drawings with bold strokes on a transparent background, 512x512 RGBA PNG.

Usage: python3 scripts/generate_view_icons.py [output directory]
The default output directory is data/icons of the repository.
"""

import math
import os
import sys

from PIL import Image, ImageDraw

SIZE   = 512
SCALE  = 4      # drawn larger and scaled down, for smooth edges
STROKE = 18
BLACK  = (0, 0, 0, 255)


class Canvas:
    def __init__(self):
        self.image = Image.new('RGBA', (SIZE * SCALE, SIZE * SCALE), (0, 0, 0, 0))
        self.draw  = ImageDraw.Draw(self.image)

    def dot(self, x, y, radius):
        s = SCALE
        self.draw.ellipse([ (x - radius) * s, (y - radius) * s, (x + radius) * s, (y + radius) * s ], fill=BLACK)

    def line(self, points, width=STROKE, round_caps=True):
        """Polyline, with round caps and round joins unless it ends at another shape."""
        s = SCALE
        self.draw.line([ (x * s, y * s) for x, y in points ], fill=BLACK, width=width * s)
        if round_caps:
            for x, y in points:
                self.dot(x, y, width / 2.0)

    def rect(self, x0, y0, x1, y1, radius=0, width=STROKE, fill=False):
        s = SCALE
        self.draw.rounded_rectangle([ x0 * s, y0 * s, x1 * s, y1 * s ], radius=radius * s,
                                    outline=BLACK, width=width * s, fill=BLACK if fill else None)

    def ellipse(self, x0, y0, x1, y1, width=STROKE):
        s = SCALE
        self.draw.ellipse([ x0 * s, y0 * s, x1 * s, y1 * s ], outline=BLACK, width=width * s)

    def polygon(self, points):
        s = SCALE
        self.draw.polygon([ (x * s, y * s) for x, y in points ], fill=BLACK)

    def clear_polygon(self, points):
        s = SCALE
        self.draw.polygon([ (x * s, y * s) for x, y in points ], fill=(0, 0, 0, 0))

    def clear_dot(self, x, y, radius):
        s = SCALE
        self.draw.ellipse([ (x - radius) * s, (y - radius) * s, (x + radius) * s, (y + radius) * s ],
                          fill=(0, 0, 0, 0))

    def save(self, path):
        self.image.resize((SIZE, SIZE), Image.LANCZOS).save(path)


def axes(c, x0=72, y0=440, x1=448, y1=72):
    """Axis cross: vertical axis at the left, horizontal axis at the bottom."""
    c.line([ (x0, y1), (x0, y0), (x1, y0) ])


def view_table(c):
    """A table with header row and cells."""
    left, top, right, bottom = 56, 88, 456, 424
    header_bottom = 176

    c.rect(left, top, right, bottom, radius=28)
    c.rect(left, top, right, header_bottom, radius=28, fill=True)
    # square lower corners of the header row
    c.rect(left, header_bottom - 40, right, header_bottom, fill=True)

    rows = 3
    row_height = (bottom - header_bottom) / rows
    for i in range(1, rows):
        y = header_bottom + i * row_height
        c.line([ (left, y), (right, y) ], width=14, round_caps=False)

    for x in (left + (right - left) / 3.0, left + 2 * (right - left) / 3.0):
        c.line([ (x, header_bottom), (x, bottom) ], width=14, round_caps=False)


def view_histogram(c):
    """Vertical bars on an axis."""
    axes(c)

    base = 440 - STROKE / 2.0
    bars = [ (120, 300), (200, 180), (280, 110), (360, 240) ]   # left edge, top
    for x, top in bars:
        c.rect(x, top, x + 56, base + STROKE, radius=6, fill=True)

    # the axis stays visible below the bars
    c.line([ (72, 440), (448, 440) ])


def view_scatterplot(c):
    """Dots in an axis cross."""
    axes(c)

    dots = [ (140, 370), (190, 300), (170, 220), (250, 330), (260, 240), (300, 170),
             (340, 280), (360, 200), (400, 130), (410, 240) ]
    for x, y in dots:
        c.dot(x, y, 22)


def view_grid(c):
    """A grid of cells with some filled cells."""
    left, top, right, bottom = 64, 64, 448, 448
    cells = 4
    step  = (right - left) / cells

    filled = { (1, 0), (2, 1), (3, 1), (0, 2), (1, 2), (2, 3) }   # column, row, no corner cell
    for col, row in filled:
        c.rect(left + col * step, top + row * step, left + (col + 1) * step, top + (row + 1) * step, fill=True)

    c.rect(left, top, right, bottom, radius=20)
    for i in range(1, cells):
        c.line([ (left + i * step, top), (left + i * step, bottom) ], width=14, round_caps=False)
        c.line([ (left, top + i * step), (right, top + i * step) ], width=14, round_caps=False)


def view_geographic(c):
    """A globe with a location mark."""
    cx, cy, r = 216, 296, 168

    c.ellipse(cx - r, cy - r, cx + r, cy + r)
    c.ellipse(cx - r * 0.45, cy - r, cx + r * 0.45, cy + r, width=14)   # meridian
    c.line([ (cx, cy - r), (cx, cy + r) ], width=14, round_caps=False)
    c.line([ (cx - r, cy), (cx + r, cy) ], width=14, round_caps=False)  # equator

    for dy in (-0.55, 0.55):                                            # parallels
        half = r * math.sqrt(1.0 - dy * dy)
        c.line([ (cx - half, cy + r * dy), (cx + half, cy + r * dy) ], width=14, round_caps=False)

    # location mark: a pin with its tip on the globe
    px, py, pr = 372, 132, 84
    tip = (px, py + 172)

    def pin(radius, tip_y, half_angle_deg):
        a = math.radians(half_angle_deg)
        return [ (px - radius * math.sin(a), py + radius * math.cos(a)),
                 (px, tip_y),
                 (px + radius * math.sin(a), py + radius * math.cos(a)) ]

    # the globe lines do not touch the pin
    c.clear_dot(px, py, pr + 22)
    c.clear_polygon(pin(pr + 22, tip[ 1 ] + 34, 62))

    c.dot(px, py, pr)
    c.polygon(pin(pr, tip[ 1 ], 62))
    c.clear_dot(px, py, 34)


ICONS = {
    'view_table.png':       view_table,
    'view_histogram.png':   view_histogram,
    'view_scatterplot.png': view_scatterplot,
    'view_grid.png':        view_grid,
    'view_geographic.png':  view_geographic,
}


def main():
    default_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', 'icons')
    out_dir     = sys.argv[ 1 ] if len(sys.argv) > 1 else default_dir

    for name, draw in ICONS.items():
        canvas = Canvas()
        draw(canvas)

        path = os.path.normpath(os.path.join(out_dir, name))
        canvas.save(path)
        print('written', path)


if __name__ == '__main__':
    main()
