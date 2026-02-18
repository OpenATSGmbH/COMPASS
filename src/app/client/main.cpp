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

#include "client.h"
#include "compass.h"
#include "logger.h"
#include "msghandler.h"
#include "util/system.h"

#include <QThread>
#include <QTimer>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <execinfo.h>

using namespace std;

void signalHandler(int signum)
{
    // write signal number (write() is async-signal-safe)
    const char msg[] = "\nCaught signal: ";
    const char nl[] = "\n";
    write(STDERR_FILENO, msg, sizeof(msg) - 1);
    char digit = '0' + signum;
    write(STDERR_FILENO, &digit, 1);
    write(STDERR_FILENO, nl, 1);

    // async-signal-safe stacktrace via glibc backtrace
    void* frames[128];
    int count = backtrace(frames, 128);
    backtrace_symbols_fd(frames, count, STDERR_FILENO);

    // invoke the default handler and process the signal
    signal(signum, SIG_DFL);
    raise(signum);
}

int main(int argc, char** argv)
{
    try
    {
        signal(SIGSEGV, signalHandler);
        signal(SIGABRT, signalHandler);
        signal(SIGTERM, signalHandler);
        
        const bool is_app_image = Utils::System::appDir() != nullptr;

        if (!is_app_image)
        {
            //localbuild => switch to xcb if on wayland (and not specified otherwise)
            if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) 
            {
                auto var = qgetenv("XDG_SESSION_TYPE");
                const char *session = var.constData();
                if (session && QString::fromLocal8Bit(session) == "wayland")
                {
                    std::cout << "setting platform to xcb" << std::endl; 
                    qputenv("QT_QPA_PLATFORM", "xcb");
                }
            }
        }

        // Enable Qt high-DPI scaling
        QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

        Client client(argc, argv);

        if (client.quitRequested())
            return 0;
            // Alternative: _exit(0) to skip static destructors entirely

        // note: do not use COMPASS::instance functions here

        if (!client.run())
        {
            COMPASS::instance().shutdown();
            return -1;
            // Alternative: _exit(1) to skip static destructors entirely
        }

        return client.exec();
        // Alternative: _exit(ret) to skip static destructors entirely
    }
    catch (std::exception& ex)
    {
        cerr << "main: caught exception '" << ex.what() << "'" << endl;

        return -1;
    }
    catch (...)
    {
        cerr << "main: caught exception" << endl;

        return -1;
    }
}
