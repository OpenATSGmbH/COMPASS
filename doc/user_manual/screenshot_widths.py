#!/usr/bin/env python3
"""
Sets the print width of the generated screenshots (images with steps in screenshots.json) in the
tex files, so that their text has about the size of the PDF body text. The scale is defaults.dpi of
screenshots.json: 121.92 DPI = 48 px/cm, measured: GUI x-height 8 px ('x' in the Filters list) =
body x-height 4.68 pt (Palladio 10 pt). Images wider than the page are capped at 19 cm and shifted
to the left. Without --apply it only lists the changes.

    python3 doc/user_manual/screenshot_widths.py [--apply]
"""

import glob, json, os, re, sys

from PIL import Image

MANUAL  = os.path.dirname(os.path.abspath(__file__))
MAX_CM  = 19.0
TEXT_CM = 14.2   # wider images get \hspace*{-(width - 14) / 2}, as the 19 cm figures with -2.5 cm

# only figures with one generated image, sized by width and framed
INCLUDE = re.compile(r'\\includegraphics\[width=([\d.]+)cm,frame\]\{([^}]*)\}')
HSPACE  = re.compile(r'^[ \t]*\\hspace\*\{[-\d.]+cm\}[ \t]*\n', re.M)


def main():
    apply = '--apply' in sys.argv
    os.chdir(MANUAL)
    spec = json.load(open('screenshots.json', encoding='utf-8'))
    px_per_cm = spec['defaults']['dpi'] / 2.54
    generated = {im['file'] for ch in spec['chapters'] for im in ch['images'] if im.get('steps')}
    generated.add('ui/figures/ui_decks.png')   # main_window.png with frames drawn on it

    changed, skipped = 0, []
    for tex in sorted(glob.glob('**/*.tex', recursive=True)):
        text = open(tex, encoding='utf-8').read()
        out, pos = [], 0
        for fig in re.finditer(r'\\begin\{figure\}.*?\\end\{figure\}', text, re.S):
            body = fig.group(0)
            path = lambda m: os.path.normpath(os.path.join(os.path.dirname(tex), m.group(2)))
            ms = [m for m in INCLUDE.finditer(body) if path(m) in generated]
            if not ms:
                continue
            if len(INCLUDE.findall(body)) != 1 or len(HSPACE.findall(body)) > 1:
                skipped.append('{}: {}'.format(tex, ms[0].group(2)))
                continue
            m = ms[0]
            width = round(min(Image.open(path(m)).size[0] / px_per_cm, MAX_CM), 1)
            new = HSPACE.sub('', body).replace(
                m.group(0), '\\includegraphics[width={:g}cm,frame]{{{}}}'.format(width, m.group(2)))
            if width > TEXT_CM:
                line = new.rfind('\n', 0, new.find('\\includegraphics')) + 1
                new = new[:line] + '  \\hspace*{{{:g}cm}}\n'.format(-round((width - 14) / 2, 1)) + new[line:]
            if new != body:
                changed += 1
                print('{}: {} -> {:g} cm'.format(tex, m.group(2), width))
                out += [text[pos:fig.start()], new]
                pos = fig.end()
        if out and apply:
            open(tex, 'w', encoding='utf-8').write(''.join(out) + text[pos:])

    print('{} figures {}'.format(changed, 'changed' if apply else 'to change'))
    for s in skipped:
        print('skipped (not one framed image): ' + s)


if __name__ == '__main__':
    main()
