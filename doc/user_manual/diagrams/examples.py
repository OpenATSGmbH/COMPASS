#!/usr/bin/env python3
"""
Two example diagrams showing the block and the sequence style of dglib.

    python3 examples.py            # writes examples/*.svg and renders *.png

The examples are not included in the manual. Real diagrams are built the
same way, but write their SVG to svg/ and render their PNG into the figures/
folder of the chapter that includes them (see make_diagrams.py).
"""

import os

from dglib import BlockDiagram, SequenceDiagram, render_png

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "examples")


def example_block():
    """Live mode data flow: where network data goes, and what displays it."""
    d = BlockDiagram("example_live_mode_flow", cols=4, rows=3, row_pitch=17.0,
                     title="in 'Live: Running' mode, network data is stored "
                           "and displayed")
    d.block("net", 0, 1, "Network lines\n'L1' to 'L4'")
    d.block("imp", 1, 1, "ASTERIX import\n(jASTERIX)")
    d.block("db", 2, 0, "Database\n(last 60 min)")
    d.block("cache", 2, 2, "Main memory\n(last 5 min)")
    d.block("geo", 3, 2, "Geographic View\n(1 s update)")
    d.arrow("net", "imp", "UDP")
    d.arrow("imp", "db", "stored")
    d.arrow("imp", "cache", "cached")
    d.arrow("cache", "geo", "shown")
    return d


def example_sequence():
    """Auto-resume from 'Live: Paused', as described in the Live Mode chapter."""
    d = SequenceDiagram("example_live_auto_resume", [
        ("user", "User"),
        ("main", "Main Window"),
        ("imp", "ASTERIX import"),
    ], title="auto-resume from 'Live: Paused'")
    d.step("user", "main", "'Pause' pressed")
    d.step("main", "imp", "switch to 'Live: Paused',\nnetwork data is cached")
    d.note("60 minutes pass ('auto_live_running_resume_ask_time')")
    d.step("main", "user", "'Resume to Live:Running' dialog")
    d.note("no action for 1 minute ('auto_live_running_resume_ask_wait_time')")
    d.step("main", "imp", "switch to 'Live: Running',\ncached data is stored")
    return d


def main():
    for build in (example_block, example_sequence):
        diagram = build()
        svg_path = diagram.write(OUT)
        print("wrote " + svg_path)
        png_path = render_png(svg_path, os.path.join(OUT, diagram.name + ".png"))
        print("rendered " + png_path)


if __name__ == "__main__":
    main()
