#!/usr/bin/env python3
"""
Takes the user manual screenshots listed in screenshots.json (next to this file) with the local
build under Xvfb, driven by runtime commands. Only images with steps are taken, by default only
the dirty ones. The images go to --out in the folder layout of the manual, plus compare/ with the
old image next to the new one. The manual is only changed with --install.

    python3 doc/user_manual/manual_screenshots.py --out ~/tmp/manual_shots [--all] [--only a.png,b.png] [--install]
"""

import argparse, io, json, os, shlex, shutil, subprocess, sys, threading, time

import msgpack
from PIL import Image, ImageDraw, ImageFilter

MANUAL = os.path.dirname(os.path.abspath(__file__))
REPO   = os.path.dirname(os.path.dirname(MANUAL))

sys.path.insert(0, os.path.join(REPO, 'experimental_src', 'py'))
from client.compass_interface import COMPASSInterface

BINARY  = os.path.join(REPO, 'build', 'bin', 'compass_client')
LICENSE = os.path.join(REPO, 'license_openats_2026.txt')
VERSION = '1.1.0'
DISPLAY = ':99'
IMAGE_TIMEOUT = 600
# fixed, because the status bar shows the DB path
BASE    = '/tmp/compass_manual'
HOME    = BASE + '/home'
DB_DIR  = BASE + '/db'


class Client:
    def __init__(self, log_fn, font, extra_args=()):
        env = dict(os.environ, HOME=HOME, DISPLAY=DISPLAY, QT_SCALE_FACTOR='1')
        for k in ('WAYLAND_DISPLAY', 'XDG_SESSION_TYPE', 'QT_QPA_PLATFORM'):
            env.pop(k, None)

        self.log  = open(log_fn, 'w')
        self.proc = subprocess.Popen([BINARY, '--open_rt_cmd_port', '--no_cfg_save', '--bundled_font', font] + list(extra_args),
                                     env=env, stdout=self.log, stderr=subprocess.STDOUT)
        self.ci = COMPASSInterface()
        for i in range(240):
            if self.proc.poll() is not None:
                raise RuntimeError('client exited with code {}'.format(self.proc.returncode))
            if self.ci.connect():
                break
            time.sleep(0.5)
        else:
            raise RuntimeError('no runtime command connection')
        self.waitFor(lambda: self.visible('mainwindow'), 'main window not visible')

    def cmd(self, c):
        r = self.ci.sendCommandAndUnpack(c)
        if not r:
            raise RuntimeError('{} failed: {}'.format(c, r.error))
        return r.value

    def tryCmd(self, c):
        r = self.ci.sendCommandAndUnpack(c)
        return r.value if r else None

    def visible(self, obj):
        v = self.tryCmd('uiget {} --visible'.format(obj))
        return isinstance(v, dict) and v.get('visible') is True

    def waitFor(self, cond, what, polls=100):
        for i in range(polls):
            if cond():
                return
            time.sleep(0.1)
        raise RuntimeError(what)

    def close(self):
        try:
            self.ci.sendCommand('quit')
            self.proc.wait(timeout=60)
        except Exception:
            pass
        if self.proc.poll() is None:
            self.proc.kill()
        self.log.close()


def seedHome():
    shutil.rmtree(HOME, ignore_errors=True)
    version_dir = os.path.join(HOME, '.compass', VERSION)
    shutil.copytree(os.path.join(REPO, 'conf'), os.path.join(version_dir, 'conf'))
    shutil.copytree(os.path.join(REPO, 'data'), os.path.join(version_dir, 'data'))
    shutil.copytree(os.path.join(REPO, 'presets'), os.path.join(version_dir, 'presets'))

    key = msgpack.unpackb(bytes.fromhex(open(LICENSE).read().strip()), raw=False)
    os.makedirs(os.path.join(HOME, '.compass', 'license'))
    json.dump({'licenses': {key['id']: key}},
              open(os.path.join(HOME, '.compass', 'license', 'licenses.json'), 'w'), indent=4)


def files(path):
    if os.path.isdir(path):
        return sorted(os.path.join(path, f) for f in os.listdir(path))
    return [path]


def escaped(obj):
    return json.dumps(obj).replace('"', '\\"')


def analyze(c, ds_type):
    # Analyze Data Source for all data sources of the DSType, against the calculated references
    sources = c.cmd('get_data_sources')
    ids  = [s['sac'] * 256 + s['sic'] for s in sources if s.get('ds_type') == ds_type]
    refs = {str(s['sac'] * 256 + s['sic']): s.get('name') == 'CalcRef'
            for s in sources if s.get('ds_type') == 'RefTraj'}
    params = {'use_data_sources': {str(i): True for i in ids},
              'use_reference_data_sources': refs, 'custom_report_name': ''}
    c.cmd('analyze_data_source --ds_type={} --config="{}"'.format(ds_type, escaped(params)))


def buildDB(name, ds, font, log_dir):
    # import with the settings of the databases entry, then reconstruction, cached in DB_DIR
    db = os.path.join(DB_DIR, name + '.db')
    if os.path.exists(db):
        return db

    os.makedirs(DB_DIR, exist_ok=True)
    tmp = db + '.build.db'
    if os.path.exists(tmp):
        os.remove(tmp)
    seedHome()
    path = lambda p: os.path.join(ds['dir'], p)
    c = Client(os.path.join(log_dir, 'build_{}.log'.format(name)), font)
    try:
        if 'context_zip' in ds:
            c.cmd('import_context {}'.format(path(ds['context_zip'])))
        else:
            c.cmd('create_context {}'.format(ds['context']))
            c.cmd('set_context {}'.format(ds['context']))
            for f in files(path(ds['data_sources'])):
                c.cmd('import_data_sources {}'.format(f))
            for f in files(path(ds['sectors'])):
                c.cmd('import_sectors_json {}'.format(f))
            if 'ffts' in ds:
                c.cmd('import_ffts_json {}'.format(path(ds['ffts'])))
        c.cmd('set_context {}'.format(ds['context']))
        c.cmd('create_db {}'.format(tmp))

        lines = {'L1': ds.get('asterix_l1', []), 'L2': ds.get('asterix_l2', [])}
        if 'asterix' in ds:
            lines['L1'] = [ds['asterix']]
        for line, entries in lines.items():
            fns = [f for e in entries for f in files(path(e))]
            if fns:
                c.cmd('import_asterix_files --filenames={} --framing=ioss --line={} --config="{}"'.format(
                    ';'.join(fns), line, escaped(ds['import'])))

        c.cmd('reconstruct_references --config="{}"'.format(
            escaped({'current_reconstructor_str': ds['reconstructor']})))

        # stored reports, e.g. for the Reports tool and the result images
        for cfg in ds.get('evaluations', []):
            params = json.load(open(path(cfg)))['parameters']
            c.cmd('evaluate --config="{}"'.format(escaped(params)))

        for ds_type in ds.get('analyses', []):
            analyze(c, ds_type)
        c.cmd('close_db')
    finally:
        c.close()
    os.rename(tmp, db)
    return db


def savePng(src, dst, dpi):
    im = Image.open(src)
    im.load()
    im.save(dst, dpi=(dpi, dpi))
    os.remove(src)


def screenBox(c, objs, visible=True):
    # objs: one object path or several joined by '+', union of their screen rects
    boxes = []
    for obj in objs.split('+'):
        obj = obj.strip('"')
        g = c.cmd('uiget {} --what screen_geometry'.format('"{}"'.format(obj) if ' ' in obj else obj))
        if isinstance(g, str):
            g = json.loads(g)
        if visible:
            g = g.get('visible') or g
        boxes.append((g['x'], g['y'], g['x'] + g['width'], g['y'] + g['height']))
    return (min(b[0] for b in boxes), min(b[1] for b in boxes), max(b[2] for b in boxes), max(b[3] for b in boxes))


def screenShot(c, objs, dst, dpi):
    box = screenBox(c, objs)
    png = subprocess.run(['import', '-display', DISPLAY, '-window', 'root', 'png:-'],
                         check=True, capture_output=True).stdout
    Image.open(io.BytesIO(png)).convert('RGB').crop(box).save(dst, dpi=(dpi, dpi))
    return box[:2]


def grabParts(c, objs, dst, dpi):
    # widgets taller than their scroll area, e.g. a report figure: each part is grabbed off-screen and placed at
    # its screen position, the gaps get the window color of the first part (its top left pixel)
    parts = []
    for obj in objs.split('+'):
        obj = obj.strip('"')
        arg = '"{}"'.format(obj) if ' ' in obj else obj
        tmp = '{}.{}.tmp.png'.format(dst, len(parts))
        c.cmd('uiget {} --what "grab:{}"'.format(arg, tmp))
        im = Image.open(tmp).convert('RGB')
        im.load()
        os.remove(tmp)
        parts.append((screenBox(c, obj, visible=False), im))
    x0 = min(p[0][0] for p in parts)
    y0 = min(p[0][1] for p in parts)
    x1 = max(p[0][2] for p in parts)
    y1 = max(p[0][3] for p in parts)
    canvas = Image.new('RGB', (x1 - x0, y1 - y0), parts[0][1].getpixel((0, 0)))
    for box, im in parts:
        canvas.paste(im, (box[0] - x0, box[1] - y0))
    canvas.save(dst, dpi=(dpi, dpi))
    return (x0, y0)


def editImage(c, step, target, origin, dpi):
    # box / blur / crop / cropw / croph <objs>: on the image taken last, in screen coordinates relative
    # to its origin, cropw and croph crop only the width or the height
    kind, objs = step.split(' ', 1)
    x0, y0, x1, y1 = screenBox(c, shlex.split(objs)[0])
    r = (x0 - origin[0], y0 - origin[1], x1 - origin[0], y1 - origin[1])
    im = Image.open(target).convert('RGB')
    if kind == 'box':
        ImageDraw.Draw(im).rectangle((r[0] - 3, r[1] - 3, r[2] + 2, r[3] + 2), outline=(255, 0, 0), width=2)
    elif kind == 'blur':
        im.paste(im.crop(r).filter(ImageFilter.GaussianBlur(6)), r[:2])
    elif kind == 'cropw':
        im = im.crop((r[0], 0, r[2], im.height))
        origin = (x0, origin[1])
    elif kind == 'croph':
        im = im.crop((0, r[1], im.width, r[3]))
        origin = (origin[0], y0)
    else:
        im = im.crop(r)
        origin = (x0, y0)
    im.save(target, dpi=(dpi, dpi))
    return origin


def compareImage(old_fn, new_fn, out_fn):
    old = Image.open(old_fn).convert('RGB')
    new = Image.open(new_fn).convert('RGB')
    m = Image.new('RGB', (old.width + new.width + 30, max(old.height, new.height) + 25), 'white')
    d = ImageDraw.Draw(m)
    d.text((5, 5), 'old {}x{}'.format(old.width, old.height), fill='red')
    d.text((old.width + 30, 5), 'new {}x{}'.format(new.width, new.height), fill='red')
    m.paste(old, (0, 25))
    m.paste(new, (old.width + 30, 25))
    os.makedirs(os.path.dirname(out_fn), exist_ok=True)
    m.save(out_fn)


def objArg(step):
    # object paths may contain spaces, e.g. "mainwindow.filter_widget_Aircraft Address"
    obj = shlex.split(step)[1]
    return '"{}"'.format(obj) if ' ' in obj else obj


def takeImage(c, img, out_dir, dpi, report, ds):
    target = os.path.join(out_dir, img['file'])
    os.makedirs(os.path.dirname(target), exist_ok=True)

    if not img.get('flight_deck', True):
        # a press on the active tool collapses the Flight Deck, the view gets the full width
        c.cmd('uiset mainwindow.toolbox "Data Sources"')
        c.cmd('uiset mainwindow.toolbox_data_sources_button ""')

    origin = None
    for step in img['steps']:
        step = step.replace('{repo}', REPO).replace('{license}', open(LICENSE).read().strip())
        if step.startswith('sleep '):
            time.sleep(int(step.split()[1]) / 1000.0)
        elif step.startswith('until '):
            # until <obj> <text>: polls the widget value until it contains the text, e.g. a progress state
            obj, text = objArg(step), shlex.split(step)[2]
            c.waitFor(lambda: text in str(c.tryCmd('uiget {}'.format(obj))), '{} did not show {}'.format(obj, text),
                      polls=3000)
        elif step.startswith('wait '):
            obj = objArg(step)
            c.waitFor(lambda: c.visible(obj), '{} did not show'.format(obj))
            time.sleep(0.5)
        elif step.startswith('analyze '):
            analyze(c, step.split()[1])
        elif step.startswith('evaluate '):
            params = json.load(open(os.path.join(ds['dir'], step.split()[1])))['parameters']
            c.cmd('evaluate --config="{}"'.format(escaped(params)))
        elif step.startswith('viewpoint '):
            # --async: the view point may open a dialog, e.g. the warning for a 3D map
            vp = json.load(open(os.path.join(MANUAL, 'screenshot_viewpoints', step.split()[1] + '.json')))
            c.cmd('set_view_point "{}"{}'.format(json.dumps(vp, separators=(',', ':')).replace('"', '\\"'),
                                                 ' --async' if step.endswith(' --async') else ''))
        elif step.startswith('try '):
            # a command that may fail, e.g. closing a warning that shows only once per session
            c.tryCmd(step[4:])
        elif step.startswith('dump '):
            report['dumps'].append({'cmd': step[5:], 'value': c.tryCmd(step[5:])})
        elif step.startswith('grab '):
            tmp = target + '.tmp.png'
            c.cmd('uiget {} --what "grab:{}"'.format(objArg(step), tmp))
            savePng(tmp, target, dpi)
            origin = screenBox(c, shlex.split(step)[1], visible=False)[:2]
        elif step.startswith('shot '):
            origin = screenShot(c, shlex.split(step)[1], target, dpi)
        elif step.startswith('grabparts '):
            origin = grabParts(c, shlex.split(step)[1], target, dpi)
        elif step.split(' ')[0] in ('box', 'blur', 'crop', 'cropw', 'croph'):
            origin = editImage(c, step, target, origin, dpi)
        else:
            c.cmd(step)
    if origin is None:
        raise RuntimeError('no grab or shot step')
    return target


def cleanup(c):
    for i in range(4):
        if not c.visible('popup'):
            break
        c.tryCmd('uiinject popup "key(Escape)"')
        time.sleep(0.3)
    for i in range(3):
        if not c.visible('dialog'):
            break
        c.tryCmd('uiset dialog "reject"')
        time.sleep(0.5)
    c.tryCmd('uiset mainwindow.toolbox "Data Sources"')


def main():
    p = argparse.ArgumentParser(description='Take the user manual screenshots listed in screenshots.json')
    p.add_argument('--out', required=True)
    p.add_argument('--list', default=os.path.join(MANUAL, 'screenshots.json'))
    p.add_argument('--all', action='store_true', help='also images that are not dirty')
    p.add_argument('--only', default='', help='comma-separated image file names, e.g. main_window.png')
    p.add_argument('--rebuild_db', action='store_true', help='build the databases of the selected images again')
    p.add_argument('--install', action='store_true', help='copy each new image over its file in the manual')
    a = p.parse_args()

    spec     = json.load(open(a.list))
    defaults = spec['defaults']
    font     = defaults['font'].split()[-1]
    dpi      = defaults['dpi']
    only     = set(a.only.split(',')) if a.only else None

    images = [i for ch in spec['chapters'] for i in ch['images']
              if i['steps'] and (a.all or i['dirty'] or only)
              and (only is None or os.path.basename(i['file']) in only or i['file'] in only)]
    # images without a database first, then per database in chapter order. A view point sets the
    # data sources to load, so the images with a view point come after the others, and images that
    # change the database state for good (order > 1) come last
    images.sort(key=lambda i: (i['database'] or '',
                               i.get('order', 1 if any(s.startswith('viewpoint ') for s in i['steps']) else 0)))

    out_dir = os.path.abspath(os.path.expanduser(a.out))
    shutil.rmtree(out_dir, ignore_errors=True)
    os.makedirs(out_dir)

    xvfb = subprocess.Popen(['Xvfb', DISPLAY, '-screen', '0', '{}x{}x24'.format(*defaults['screen_size']),
                             '-dpi', '96', '-nolisten', 'tcp'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)
    results = []
    c = None
    try:
        dbs = {}
        for name in sorted({i['database'] for i in images if i['database']}):
            if a.rebuild_db and os.path.exists(os.path.join(DB_DIR, name + '.db')):
                os.remove(os.path.join(DB_DIR, name + '.db'))
            t0 = time.time()
            built = buildDB(name, spec['databases'][name], font, out_dir)
            dbs[name] = os.path.join(BASE, os.path.basename(built))
            shutil.copy(built, dbs[name])
            print('db {} ({:.0f} s)'.format(built, time.time() - t0), flush=True)

        seedHome()
        c = Client(os.path.join(out_dir, 'client.log'), font, defaults.get('client_args', []))
        open_db = None
        for img in images:
            # fresh: reopen the DB, so no data is loaded, e.g. for an empty view
            if img['database'] != open_db or (img.get('fresh') and open_db):
                if open_db:
                    c.cmd('close_db')
                if img['database']:
                    c.cmd('open_db {} --assure_open'.format(dbs[img['database']]))
                open_db = img['database']
            report = {'file': img['file'], 'dumps': []}
            t0 = time.time()
            # a modal dialog inside a command blocks the command queue for good: kill the client after
            # IMAGE_TIMEOUT, the next image gets a new one
            watchdog = threading.Timer(IMAGE_TIMEOUT, c.proc.kill)
            watchdog.start()
            try:
                new = takeImage(c, img, out_dir, dpi, report, spec['databases'].get(img['database']))
                report['ok'] = True
                old = os.path.join(MANUAL, img['file'])
                if os.path.exists(old):
                    compareImage(old, new, os.path.join(out_dir, 'compare', img['file']))
                if a.install:
                    shutil.copy(new, old)
            except Exception as e:
                report['ok'] = False
                report['error'] = str(e)
            watchdog.cancel()
            if c.proc.poll() is not None:
                report['ok'] = False
                report['error'] = report.get('error', '') + ' (client ended, restarted)'
                c.close()
                c = Client(os.path.join(out_dir, 'client_{}.log'.format(len(results))), font,
                           defaults.get('client_args', []))
                open_db = None
                results.append(report)
                print('{:55s} ERR {}'.format(img['file'], report['error']), flush=True)
                continue
            cleanup(c)
            report['seconds'] = round(time.time() - t0, 1)
            results.append(report)
            print('{:55s} {} {}'.format(img['file'], 'ok ' if report['ok'] else 'ERR', report.get('error', '')),
                  flush=True)
    finally:
        if c:
            c.close()
        xvfb.terminate()
        json.dump(results, open(os.path.join(out_dir, 'results.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
