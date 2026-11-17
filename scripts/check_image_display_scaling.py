#!/usr/bin/env python3
"""
Check that a rendered chart image does not depend on the display scaling.

Runs the hidden unit test "[displayscaling]" of compass_tests with QT_SCALE_FACTOR 1 and 2 and
compares the two images (2400 x 1600 pixels). They must be equal.

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


def render(binary, platform, factor, file_name):
    env = dict(os.environ)
    env['QT_QPA_PLATFORM'] = platform
    env['QT_SCALE_FACTOR'] = factor
    env['COMPASS_TEST_IMAGE_FILE'] = file_name

    result = subprocess.run([binary, '[displayscaling]'], env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, timeout=300)

    if result.returncode != 0 or not os.path.isfile(file_name):
        print(result.stdout)
        sys.exit('rendering with display scaling {} failed'.format(factor))

    with open(file_name, 'rb') as f:
        return f.read()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--build-dir', default='build', help='build directory with bin/compass_tests')
    parser.add_argument('--platform', required=True, help='Qt platform, e.g. wayland or xcb')
    parser.add_argument('--keep', help='directory to keep the two images in')
    args = parser.parse_args()

    binary = os.path.join(args.build_dir, 'bin', 'compass_tests')
    if not os.path.isfile(binary):
        sys.exit('{} not found'.format(binary))

    with tempfile.TemporaryDirectory() as tmp_dir:
        out_dir = args.keep or tmp_dir
        os.makedirs(out_dir, exist_ok=True)

        images = [ render(binary, args.platform, factor, os.path.join(out_dir, 'display_scaling_{}.png'.format(factor)))
                   for factor in ('1', '2') ]

    if images[ 0 ] != images[ 1 ]:
        sys.exit('display scaling check, platform {}: the images DIFFER'.format(args.platform))

    print('display scaling check, platform {}: the images are equal'.format(args.platform))


if __name__ == '__main__':
    main()
