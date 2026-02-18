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

#include <iostream>

#include <QApplication>
#include <QProcess>
#include <QFileInfo>
#include <QMessageBox>
#include <QTextStream>

int main(int argc, char** argv)
{
    QApplication app(argc, argv);  // GUI app to support QMessageBox

    QProcess *process = new QProcess();

    // path to client (assumed to be in the same directory)
    QString realAppPath = QCoreApplication::applicationDirPath() + "/compass_client";

    //collect command line args for COMPASS client
    bool has_rt_cmd_port = false;
    QStringList client_args;
    for (int i = 1; i < argc; ++i) 
    {
        QString arg = QString::fromLocal8Bit(argv[i]);
        if (arg == "--open_rt_cmd_port")
            has_rt_cmd_port = true;
        client_args << arg;
    }

    // ensure executable exists
    if (!QFileInfo::exists(realAppPath)) 
    {
        QString err = "COMPASS client not found";
        QTextStream(stderr) << err << "\n";
        if (!has_rt_cmd_port)
            QMessageBox::critical(nullptr, "Error", err);
        return -1;
    }

    //@TODO: forwarding not working under windows
    process->setProcessChannelMode(QProcess::ForwardedChannels);

    // connect to output
    // QObject::connect(process, &QProcess::readyReadStandardOutput, [process]() {
    //     QTextStream ts(stdout);
    //     ts << process->readAllStandardOutput();
    //     ts.flush();
    // });

    // connect to output
    // QObject::connect(process, &QProcess::readyReadStandardError, [process]() {
    //     QTextStream ts(stdout);
    //     ts << process->readAllStandardError();
    //     ts.flush();
    // });

    // monitor exit status
    QObject::connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                     [process, &has_rt_cmd_port](int exitCode, QProcess::ExitStatus status) {
        if (status == QProcess::CrashExit) 
        {
            QString err = "COMPASS client crashed with code " + QString::number(exitCode);
            QTextStream(stderr) << err << "\n";
            if (!has_rt_cmd_port)
                QMessageBox::critical(nullptr, "Application Crash", err);
        } 
        else 
        {
            QTextStream(stdout) << "COMPASS client exited normally with code " << exitCode << "\n";
        }
        QCoreApplication::exit(exitCode);
    });

    // start client
    process->start(realAppPath, client_args, QIODevice::ReadWrite);
    if (!process->waitForStarted()) 
    {
        QString err = "Could not start COMPASS client:\n" + process->errorString();
        QTextStream(stderr) << err << "\n";
        if (!has_rt_cmd_port)
            QMessageBox::critical(nullptr, "Error", err);
        return -1;
    }

    return app.exec();
}
