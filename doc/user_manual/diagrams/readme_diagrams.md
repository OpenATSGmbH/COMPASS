# User Manual Diagrams

Screenshots show the UI, diagrams show a concept: a data flow, a sequence of
steps, a time line. Diagrams of the user manual are **generated from Python
scripts**, not drawn by hand, so they share one style and can be corrected
later by editing a few lines.

Two rules decide what a diagram looks like:

- **Prefer simple block diagrams.** Components as blocks, the data path or
  the flow as arrows, and nothing else. This is the form for component
  diagrams, data path and flow diagrams, and mode overviews. A sequence
  diagram is the exception, only where the order of steps is the point, and
  then only as steps and plain text lines.
- **Do not draw UML-like diagrams.** No state machines with guards and
  labeled transitions, no sequence diagrams with actors, activation bars and
  frames, no class or component notation. They are too complex for the
  manual. What a block is (a file, a task, a window) is said by its label,
  not by a shape, a stereotype or a color; what an arrow means is said by
  the text next to the figure, and only where needed by a short label.

A diagram is a few labeled blocks, thin arrows, plain text and dashed guide
lines. If it needs more than that, it is too complex: split it, or describe
it in the text.

## Folder

```
doc/user_manual/diagrams/
  readme_diagrams.md     this file
  dglib.py               library: BlockDiagram, SequenceDiagram, render_png
  make_diagrams.py       the time line diagrams of the ASTERIX import chapter
  live_mode.py           the diagrams of the Live Mode chapter
  examples.py            one block and one sequence example
  examples/              the rendered examples (not included in the manual)
  svg/                   SVG sources of the manual's diagrams (checked in)
```

The PNG of a diagram is rendered into the `figures/` folder of the chapter
that includes it, next to the screenshots. SVG and PNG are both checked in,
so the manual builds without the scripts.

Requirements: `python3` and `inkscape` (for the PNG rendering).

## Workflow

1. Write a builder function for the diagram, in `make_diagrams.py` for a time
   line, or in a script of the chapter's topic for a block or sequence
   diagram. Copy the closest existing function.
2. Verify every label against the code, as for any manual text: UI strings in
   single quotes exactly as the application shows them, parameter names and
   defaults as registered.
3. Run the script. It writes `svg/<name>.svg` and renders the PNG:

   ```bash
   cd doc/user_manual/diagrams
   python3 examples.py                  # the two examples
   python3 make_diagrams.py             # the time lines
   python3 make_diagrams.py --svg-only  # SVG only, no Inkscape needed
   ```

   In `make_diagrams.py` every diagram is one function, listed in `DIAGRAMS`
   at the end of the script.

4. Read the warnings. The library warns when a label does not fit its block
   or its arrow. Shorten the label, do not accept a reduced font.
5. Look at the PNG. No text may sit on a line, nothing may overlap.
6. Include it in the chapter, at 15 cm width and **without** `frame` (the
   frame is for screenshots):

   ```latex
   \begin{figure}[H]
     \center
       \includegraphics[width=15cm]{figures/<name>.png}
     \caption{...}
   \end{figure}
   ```

## Style

Defined once at the top of `dglib.py` and `make_diagrams.py`. New diagrams
must not add to it.

| Element | Style |
| - | - |
| Canvas | 160 mm wide, user unit 1 mm, white background, no outer frame |
| Font | DejaVu Sans, matching the Qt screenshots |
| Text, arrows | `#131334` (compass_dark) |
| Block | border `#3B3AA7` (compass_medium), fill `#DCDCF7`, corner radius 1.2 mm |
| Guide lines | `#9897EE` (compass_light), dashed: lifelines, day boundaries |
| Problem marker | `#A02020` (dark red), fill `#F2D9D9`, dashed border |
| Sizes | blocks 3.2, arrow labels 2.7, notes and statement line 2.7 |

The three blues are the colors of the OpenATS logo. Dark red is used for
problem cases only.

Rules:

- As few elements as possible. Three blocks and five arrows are a good
  diagram; a label on every arrow, a statement line and a group box on top
  of that are not.
- Color is never the only carrier of information. Every block and every
  marker has a text label. An arrow gets a label only where the text next to
  the figure does not already say what it is.
- Text never sits on a line. Arrow labels go inside their segment, the
  sequence diagram interrupts a lifeline behind a label that crosses it.
- Labels are short phrases in sentence case, American English, no trailing
  period. UI strings in single quotes as in the manual text ('Pause'). A
  newline in a label starts a second line. Manual vocabulary applies (see
  `readme_user_manual.md`).
- One block style. Do not invent shapes, fills or icons for kinds of blocks.
- A statement line above the diagram is optional. The time lines use it for
  the result of a case; a figure whose caption says what it shows does not
  need one.
- Block diagrams: data flows left to right, or top to bottom. At most 4
  columns by 3 rows.
- Sequence diagrams: participants across the top as blocks, time runs
  downward, one labeled arrow per step. Passing time, conditions and results
  are plain text lines between the steps, not frames.
- Time lines: time runs left to right on one arrow axis, recording files are
  blocks above it, one row per file. A broken axis (two segments) when the
  interesting parts are minutes apart inside a 24 h span, labeled '(axis not
  to scale)'.

## dglib.py

### Block diagram

```python
from dglib import BlockDiagram, render_png

d = BlockDiagram("live_mode_flow", cols=4, rows=3,
                 title="in 'Live: Running' mode, network data is stored and displayed")
d.block("net", 0, 1, "Network lines\n'L1' to 'L4'")      # key, column, row, label
d.block("imp", 1, 1, "ASTERIX import\n(jASTERIX)")
d.block("db", 2, 0, "Database\n(last 60 min)")
d.block("cache", 2, 2, "Main memory\n(last 5 min)")
d.block("geo", 3, 2, "Geographic View\n(1 s update)")
d.arrow("net", "imp", "UDP")                             # src, dst, label
d.arrow("imp", "db", "stored")
d.arrow("imp", "cache", "cached")
d.arrow("cache", "geo", "shown")
svg = d.write("svg")
render_png(svg, "../live/figures/live_mode_flow.png")
```

- `BlockDiagram(name, cols, rows, row_pitch=17.0, block_h=11.0, block_w=None,
  title=None)`: a grid of cells, one block per cell, the column pitch follows
  from the canvas width.
- `block(key, col, row, label, span=1, alert=False)`: `span` merges columns,
  `alert` draws the dark red problem marker.
- `arrow(src, dst, label=None, route=None, label_side=None, offset=0.0)`:
  the route is chosen from the positions ('h' horizontal, 'v' vertical, 'vh'
  out top or bottom then sideways, 'hv', 'hvh'); 'd' is a straight line at
  any angle, for a triangle of blocks. The label goes next to the longest
  segment it fits: above a horizontal one, right of a vertical one;
  `label_side='below'` or `'left'` flips that. `offset` shifts an 'h' arrow
  up or down, a 'v' arrow left or right and a 'd' arrow sideways by that
  many mm, for two arrows between the same blocks, one per direction (see
  `live_mode.py`).
- `note(col, row, label)`: plain text on a cell, fractions allowed.

### Sequence diagram

```python
from dglib import SequenceDiagram, render_png

d = SequenceDiagram("live_auto_resume",
                    [("user", "User"), ("main", "Main Window"), ("imp", "ASTERIX import")],
                    title="auto-resume from 'Live: Paused'")
d.step("user", "main", "'Pause' pressed")
d.step("main", "imp", "switch to 'Live: Paused',\nnetwork data is cached")
d.note("60 minutes pass ('auto_live_running_resume_ask_time')")
d.step("main", "user", "'Resume to Live:Running' dialog")
d.note("no action for 1 minute ('auto_live_running_resume_ask_wait_time')")
d.step("main", "imp", "switch to 'Live: Running',\ncached data is stored")
svg = d.write("svg")
render_png(svg, "../live/figures/live_auto_resume.png")
```

- `SequenceDiagram(name, participants, row=9.0, title=None)`: participants as
  `(key, label)`, left to right in the order of first use.
- `step(src, dst, label)`: one labeled arrow, one row down.
- `note(label, key=None, side='right')`: a plain text line, centered across
  the diagram or beside the lifeline of `key`.
- `gap(factor=0.5)`: extra vertical space in rows.

The rendered examples are in `examples/`, built by `examples.py`.
