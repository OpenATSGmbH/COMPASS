#!/usr/bin/env python3
"""
Check that the rendered chart images do not depend on the display scaling.

Runs the hidden unit tests "[displayscaling]" of compass_tests with QT_SCALE_FACTOR 1 and 2. Each test
writes its chart images (2400 x 1600 pixels) into a directory: a histogram and scatterplots with a
rotated axis title, own tick labels, rotated date time labels and a legend. The images of both runs
must be equal.

Run it on a platform with real font settings, e.g. --platform wayland or --platform xcb. No window is
shown. On the offscreen platform the images are equal also with fonts that depend on the display
scaling, so the check proves nothing there.

Usage: python3 scripts/check_image_display_scaling.py --build-dir build --platform wayland
"""
import argparse
import os
import subprocess
import sys
import tempfile


def render(binary, platform, factor, out_dir):
    """Returns the images of one run, file name -> content."""
    os.makedirs(out_dir, exist_ok=True)

    env = dict(os.environ)
    env['QT_QPA_PLATFORM'] = platform
    env['QT_SCALE_FACTOR'] = factor
    env['COMPASS_TEST_IMAGE_DIR'] = out_dir

    result = subprocess.run([binary, '[displayscaling]'], env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, timeout=300)

    if result.returncode != 0:
        print(result.stdout)
        sys.exit('rendering with display scaling {} failed'.format(factor))

    images = {}
    for name in sorted(os.listdir(out_dir)):
        if name.endswith('.png'):
            with open(os.path.join(out_dir, name), 'rb') as f:
                images[name] = f.read()

    if not images:
        print(result.stdout)
        sys.exit('rendering with display scaling {} wrote no image'.format(factor))

    return images


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--build-dir', default='build', help='build directory with bin/compass_tests')
    parser.add_argument('--platform', required=True, help='Qt platform, e.g. wayland or xcb')
    parser.add_argument('--keep', help='directory to keep the images in (one sub directory per display scaling)')
    args = parser.parse_args()

    binary = os.path.join(args.build_dir, 'bin', 'compass_tests')
    if not os.path.isfile(binary):
        sys.exit('{} not found'.format(binary))

    with tempfile.TemporaryDirectory() as tmp_dir:
        out_dir = os.path.abspath(args.keep or tmp_dir)

        images = [render(binary, args.platform, factor, os.path.join(out_dir, 'scale_{}'.format(factor)))
                  for factor in ('1', '2')]

    names = sorted(set(images[0]) | set(images[1]))
    differing = [name for name in names if images[0].get(name) != images[1].get(name)]

    for name in names:
        print('{}: {}'.format(name, 'DIFFER' if name in differing else 'equal'))

    if differing:
        sys.exit('display scaling check, platform {}: {} of {} images DIFFER'.format(
            args.platform, len(differing), len(names)))

    print('display scaling check, platform {}: the {} images are equal'.format(args.platform, len(names)))


if __name__ == '__main__':
    main()
