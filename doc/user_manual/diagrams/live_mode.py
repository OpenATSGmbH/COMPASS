#!/usr/bin/env python3
"""
Diagrams of the Live Mode chapter (live/live.tex).

    python3 live_mode.py            # writes svg/*.svg and renders the PNGs

One function per diagram, listed in DIAGRAMS. The SVG sources go to svg/,
the PNGs into ../live/figures/. Both are checked in.
"""

import math
import os

from dglib import BlockDiagram, render_png, MARGIN, W_TOTAL

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_SVG = os.path.join(HERE, "svg")
OUT_PNG = os.path.normpath(os.path.join(HERE, "..", "live", "figures"))


def modes():
    """The three application modes and the switches between them, as an
    equilateral triangle: 'Offline' on top, the two live modes at the base.
    Rendered as figures/modes.png. The text next to the figure names the
    actions."""
    side = 60.0                                  # block center to center, mm
    block_h, row_pitch = 11.0, 17.0
    col_pitch = (W_TOTAL - 2 * MARGIN) / 3.0
    half = side / 2.0 / col_pitch                # half base, in columns
    height = side * math.sqrt(3.0) / 2.0 / row_pitch   # triangle height, in rows
    rows = height + 0.5 + block_h / 2.0 / row_pitch + 0.1
    d = BlockDiagram("live_modes", cols=3, rows=rows, row_pitch=row_pitch,
                     block_h=block_h, block_w=30.0)
    d.block("offline", 1.0, 0.0, "Offline")
    d.block("paused", 1.0 - half, height, "Live: Paused")
    d.block("live", 1.0 + half, height, "Live: Running")
    d.arrow("offline", "live", route="d", offset=-2.0)
    d.arrow("live", "offline", route="d", offset=-2.0)
    d.arrow("live", "paused", offset=-2.5)
    d.arrow("paused", "live", offset=2.5)
    d.arrow("paused", "offline", route="d")
    return d, "modes"


DIAGRAMS = [modes]


def main():
    for build in DIAGRAMS:
        diagram, png_name = build()
        svg_path = diagram.write(OUT_SVG)
        print("wrote " + svg_path)
        png_path = render_png(svg_path, os.path.join(OUT_PNG, png_name + ".png"))
        print("rendered " + png_path)


if __name__ == "__main__":
    main()
