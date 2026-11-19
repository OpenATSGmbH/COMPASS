#!/usr/bin/env python3
"""
Takes the user manual screenshots listed in screenshots.json (next to this file) with the local
build under Xvfb, driven by runtime commands. Only images with steps are taken, by default only
the dirty ones. The images go to --out in the folder layout of the manual, plus compare/ with the
old image next to the new one. The manual itself is not changed.

    python3 doc/user_manual/manual_screenshots.py --out ~/tmp/manual_shots [--all] [--only a.png,b.png]
"""

import argparse, io, json, os, shlex, shutil, subprocess, sys, time

import msgpack
from PIL import Image, ImageDraw

MANUAL = os.path.dirname(os.path.abspath(__file__))
REPO   = os.path.dirname(os.path.dirname(MANUAL))

sys.path.insert(0, os.path.join(REPO, 'experimental_src', 'py'))
from client.compass_interface import COMPASSInterface

BINARY  = os.path.join(REPO, 'build', 'bin', 'compass_client')
LICENSE = os.path.join(REPO, 'license_openats_2026.txt')
VERSION = '1.1.0'
DISPLAY = ':99'
# fixed, because the status bar shows the DB path
BASE    = '/tmp/compass_manual'
HOME    = BASE + '/home'
DB_DIR  = BASE + '/db'


class Client:
    def __init__(self, log_fn, font):
        env = dict(os.environ, HOME=HOME, DISPLAY=DISPLAY, QT_SCALE_FACTOR='1')
        for k in ('WAYLAND_DISPLAY', 'XDG_SESSION_TYPE', 'QT_QPA_PLATFORM'):
            env.pop(k, None)

        self.log  = open(log_fn, 'w')
        self.proc = subprocess.Popen([BINARY, '--open_rt_cmd_port', '--no_cfg_save', '--bundled_font', font],
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


def screenShot(c, obj, dst, dpi):
    g = c.cmd('uiget {} --what screen_geometry'.format(obj))
    if isinstance(g, str):
        g = json.loads(g)
    png = subprocess.run(['import', '-display', DISPLAY, '-window', 'root', 'png:-'],
                         check=True, capture_output=True).stdout
    im = Image.open(io.BytesIO(png)).convert('RGB')
    im.crop((g['x'], g['y'], g['x'] + g['width'], g['y'] + g['height'])).save(dst, dpi=(dpi, dpi))


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


def takeImage(c, img, out_dir, dpi, report):
    target = os.path.join(out_dir, img['file'])
    os.makedirs(os.path.dirname(target), exist_ok=True)

    if not img.get('flight_deck', True):
        # a press on the active tool collapses the Flight Deck, the view gets the full width
        c.cmd('uiset mainwindow.toolbox "Data Sources"')
        c.cmd('uiset mainwindow.toolbox_data_sources_button ""')

    taken = False
    for step in img['steps']:
        if step.startswith('sleep '):
            time.sleep(int(step.split()[1]) / 1000.0)
        elif step.startswith('wait '):
            obj = objArg(step)
            c.waitFor(lambda: c.visible(obj), '{} did not show'.format(obj))
            time.sleep(0.5)
        elif step.startswith('dump '):
            report['dumps'].append({'cmd': step[5:], 'value': c.tryCmd(step[5:])})
        elif step.startswith('grab '):
            tmp = target + '.tmp.png'
            c.cmd('uiget {} --what "grab:{}"'.format(objArg(step), tmp))
            savePng(tmp, target, dpi)
            taken = True
        elif step.startswith('shot '):
            screenShot(c, objArg(step), target, dpi)
            taken = True
        else:
            c.cmd(step)
    if not taken:
        raise RuntimeError('no grab or shot step')
    return target


def cleanup(c):
    if c.visible('popup'):
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
    p.add_argument('--rebuild_db', action='store_true')
    a = p.parse_args()

    spec     = json.load(open(a.list))
    defaults = spec['defaults']
    font     = defaults['font'].split()[-1]
    dpi      = defaults['dpi']
    only     = set(a.only.split(',')) if a.only else None

    images = [i for ch in spec['chapters'] for i in ch['images']
              if i['steps'] and (a.all or i['dirty'] or only)
              and (only is None or os.path.basename(i['file']) in only or i['file'] in only)]
    # images without a database first, then per database in chapter order
    images.sort(key=lambda i: (i['database'] or ''))

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
        if a.rebuild_db:
            shutil.rmtree(DB_DIR, ignore_errors=True)
        dbs = {}
        for name in sorted({i['database'] for i in images if i['database']}):
            t0 = time.time()
            built = buildDB(name, spec['databases'][name], font, out_dir)
            dbs[name] = os.path.join(BASE, os.path.basename(built))
            shutil.copy(built, dbs[name])
            print('db {} ({:.0f} s)'.format(built, time.time() - t0), flush=True)

        seedHome()
        c = Client(os.path.join(out_dir, 'client.log'), font)
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
            try:
                new = takeImage(c, img, out_dir, dpi, report)
                report['ok'] = True
                old = os.path.join(MANUAL, img['file'])
                if os.path.exists(old):
                    compareImage(old, new, os.path.join(out_dir, 'compare', img['file']))
            except Exception as e:
                report['ok'] = False
                report['error'] = str(e)
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
