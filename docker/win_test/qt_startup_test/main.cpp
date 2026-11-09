/*
 * This file is part of OpenATS COMPASS.
 *
 * COMPASS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * COMPASS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with COMPASS. If not, see <http://www.gnu.org/licenses/>.
 */

// Windows build test: rare crash in the QApplication constructor (QWindowsContextPrivate)
// at COMPASS startup on crunch_win. Does the same steps as the COMPASS main() up to the
// QApplication constructor, without the COMPASS libraries. If this program also crashes in
// a restart loop, the cause is in Qt, the driver or Windows, not in COMPASS.
// Command line arguments are ignored (the restart loop passes the COMPASS arguments).

#include <QApplication>
#include <QTimer>

#include <cstdio>
#include <iostream>

int main(int argc, char** argv)
{
    // as in COMPASS main()
    setvbuf(stdout, nullptr, _IOLBF, 0);
    std::cout << "qt_startup_test: start" << std::endl;

    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QApplication app(argc, argv);

    std::cout << "qt_startup_test: QApplication created" << std::endl;

    // short event loop, then quit
    QTimer::singleShot(200, &app, &QApplication::quit);
    int ret = app.exec();

    std::cout << "qt_startup_test: done" << std::endl;
    return ret;
}
