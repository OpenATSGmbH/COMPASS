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
from make_diagrams import Diagram as TimeLine

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


def flow():
    """Where the live data goes in 'Live: Running': every network line is
    decoded by the ASTERIX import and stored in the database; the toggled
    lines, restricted by the filters, are also kept in main memory, which the
    Geographic View displays. Rendered as figures/live_flow.png."""
    d = BlockDiagram("live_flow", cols=4, rows=3, block_w=30.0)
    d.block("net", 0, 1, "Network lines\nL1 to L4")
    d.block("imp", 1, 1, "ASTERIX import")
    d.block("db", 2, 0, "Database\nlast 60 min")
    d.block("ram", 2, 2, "RAM\nlast 5 min")
    d.block("geo", 3, 2, "Geographic View\n1 s update")
    d.arrow("net", "imp", "UDP")
    d.arrow("imp", "db", "all data", route="vh")
    d.arrow("imp", "ram", route="vh")
    d.arrow("ram", "geo")
    # the label is too wide for the arrow's horizontal run (23 mm), so it is
    # set as a note just below that run, clear of the vertical segment and of
    # the RAM block
    d.note(1.132, 2.288, "selected data sources,\nlines, applied filters")
    return d, "live_flow"


def data_windows():
    """The two time windows of the live data on one time line: the database
    holds the last 60 minutes, main memory the last 5 minutes. Data within
    the last 5 minutes is inspected in 'Live: Running' with the time
    scrollbar, older data in 'Live: Paused' with a load from the database.
    Times are minutes before now, the axis is broken between the two parts.
    No statement line above the figure. Rendered as
    figures/live_data_windows.png."""
    d = TimeLine("live_data_windows", [(-60.0, -15.0), (-6.0, 0.0)], 2)
    d.block(0, -60.0, 0.0, "Database: last 60 min", "60 min ago", "now")
    d.block(1, -5.0, 0.0, "RAM: last 5 min", "5 min ago", None)
    d.tick(-35.0, "35 min ago", "'Live: Paused', load")
    d.tick(-3.0, "3 min ago", "'Live: Running'")
    d.note((4.0 + W_TOTAL - 14.0) / 2.0, d.axis_y + 8.0, "(axis not to scale)")
    d.axis()
    return d, "live_data_windows"


DIAGRAMS = [modes, flow, data_windows]


def main():
    for build in DIAGRAMS:
        diagram, png_name = build()
        if isinstance(diagram, TimeLine):
            svg_path = diagram.write()           # writes into svg/ itself
        else:
            svg_path = diagram.write(OUT_SVG)
        print("wrote " + svg_path)
        png_path = render_png(svg_path, os.path.join(OUT_PNG, png_name + ".png"))
        print("rendered " + png_path)


if __name__ == "__main__":
    main()
