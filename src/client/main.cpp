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

#include <QThread>
#include <QTimer>

#include <osgEarth/Registry>

#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/stacktrace.hpp>

#include <iostream>
#include <signal.h>

using namespace std;

void signalHandler(int signum) 
{
    std::cerr << "Caught signal " << signum << std::endl;

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
        
        const bool is_app_image = getenv("APPDIR") != nullptr;

        if (!is_app_image)
        {
            //localbuild => switch to xcb if on wayland (and not specified otherwise)
            if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) 
            {
                const char *session = qgetenv("XDG_SESSION_TYPE").constData();
                if (session && QString::fromLocal8Bit(session) == "wayland")
                {
                    std::cout << "setting platform to xcb" << std::endl; 
                    qputenv("QT_QPA_PLATFORM", "xcb");
                }
            }
        }

        // Enable Qt high-DPI scaling
        QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

        // 1) Force-initialize the GDAL mutex (and register its atexit-hook)
        //osgEarth::getGDALMutex();

        // 2) Then initialize the Registry (which registers its destructor next)
        //osgEarth::Registry::instance();

        Client client(argc, argv);

        if (client.quitRequested())
            return 0;

        // note: do not use COMPASS::instance functions here

        if (!client.run())
        {
            // // process events a bit to allow for correct cleanup
            // auto start_time = boost::posix_time::microsec_clock::local_time();
            // while ((boost::posix_time::microsec_clock::local_time() - start_time).total_milliseconds()
            //         < 50)
            // {
            //     QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
            //     QThread::msleep(1);
            // }
            COMPASS::instance().shutdown();

            return -1;
        }

        return client.exec();
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
