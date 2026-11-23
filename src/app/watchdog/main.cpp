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
#include <QDesktopServices>
#include <QDir>
#include <QIcon>
#include <QProcess>
#include <QPushButton>
#include <QFileInfo>
#include <QMessageBox>
#include <QTextStream>
#include <QUrl>

/**
 * Log folder of the client, ~/.compass/<version>/log/ (HOME_LOG_DIRECTORY in files.h).
 */
static QString logDirectory()
{
    return QDir::homePath() + "/.compass/" + COMPASS_VERSION + "/log";
}

/**
 * Log file of the client process: compass_<start time>_<pid>.log (Logger::logFilePath), empty if not found.
 */
static QString logFileOfClient(qint64 pid)
{
    if (pid <= 0)
        return {};

    QDir dir(logDirectory());
    const QStringList files = dir.entryList({ "compass_*_" + QString::number(pid) + ".log" }, QDir::Files, QDir::Name);

    return files.isEmpty() ? QString() : dir.absoluteFilePath(files.last());
}

/**
 * Crash message with buttons to open the log file and the log folder.
 */
static void showCrashDialog(const QString& err, const QString& log_file)
{
    QMessageBox msg(QMessageBox::Critical, "Application Crash", err);

    QString text = err;
    if (!log_file.isEmpty())
        text += "\n\nThe log file contains a crash report:\n" + QDir::toNativeSeparators(log_file);
    else
        text += "\n\nThe log files are in the folder:\n" + QDir::toNativeSeparators(logDirectory());
    msg.setText(text);
    msg.setTextInteractionFlags(Qt::TextSelectableByMouse);

    QPushButton* open_file = nullptr;
    if (!log_file.isEmpty())
    {
        open_file = msg.addButton("Open Log File", QMessageBox::ActionRole);
        open_file->setIcon(QIcon());
        open_file->setToolTip("Opens the log file with the crash report");
    }

    QPushButton* open_folder = msg.addButton("Open Log Folder", QMessageBox::ActionRole);
    open_folder->setIcon(QIcon());
    open_folder->setToolTip("Opens the folder of the log files");

    QPushButton* close = msg.addButton("Close", QMessageBox::RejectRole);
    close->setIcon(QIcon());

    //the open buttons keep the message open
    while (true)
    {
        msg.exec();

        if (open_file && msg.clickedButton() == open_file)
            QDesktopServices::openUrl(QUrl::fromLocalFile(log_file));
        else if (msg.clickedButton() == open_folder)
            QDesktopServices::openUrl(QUrl::fromLocalFile(logDirectory()));
        else
            break;
    }
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);  // GUI app to support QMessageBox

    QProcess *process = new QProcess();

    // path to client (assumed to be in the same directory)
    QString realAppPath = QCoreApplication::applicationDirPath() + "/compass_client";
#ifdef _WIN32
    realAppPath += ".exe";
#endif

    //collect command line args for COMPASS client
    bool has_rt_cmd_port = false;
    QStringList client_args;
    // QCoreApplication::arguments: correct encoding on all platforms (wide arguments on Windows)
    const QStringList app_args = QCoreApplication::arguments();
    for (int i = 1; i < app_args.size(); ++i) 
    {
        const QString& arg = app_args.at(i);
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
            QMessageBox::critical(QApplication::activeWindow(), "Error", err);
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

    // process id of the client, for its log file (no longer available from the process when it ended)
    qint64 client_pid = 0;

    // monitor exit status
    QObject::connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                     [process, &has_rt_cmd_port, &client_pid](int exitCode, QProcess::ExitStatus status) {
        int relay_code = exitCode;
        if (status == QProcess::CrashExit)
        {
#ifdef _WIN32
            // Windows: the exit code is the NTSTATUS of the crash (e.g. 0xC0000005 access
            // violation), never 0 - relay it unchanged
            QString err = "COMPASS client crashed with code 0x"
                        + QString::number(static_cast<quint32>(exitCode), 16).toUpper();
#else
            // Shell convention: signal-death encoded as 128 + signum.
            // Qt's exitCode on CrashExit is often 0 on Unix, which would
            // collide with a clean exit - force the high bit so the
            // Python test framework can distinguish via proc.returncode.
            relay_code = 128 + (exitCode > 0 ? exitCode : 6 /* SIGABRT fallback */);
            QString err = "COMPASS client crashed with code " + QString::number(exitCode)
                        + " (relaying " + QString::number(relay_code) + ")";
#endif
            const QString log_file = logFileOfClient(client_pid);

            QTextStream(stderr) << err << "\n"
                                << "Log file: " << QDir::toNativeSeparators(log_file.isEmpty() ? logDirectory() : log_file) << "\n";
            if (!has_rt_cmd_port)
                showCrashDialog(err, log_file);
        }
        else
        {
            QTextStream(stdout) << "COMPASS client exited normally with code " << exitCode << "\n";
        }
        QCoreApplication::exit(relay_code);
    });

    // start client
    process->start(realAppPath, client_args, QIODevice::ReadWrite);
    if (!process->waitForStarted()) 
    {
        QString err = "Could not start COMPASS client:\n" + process->errorString();
        QTextStream(stderr) << err << "\n";
        if (!has_rt_cmd_port)
            QMessageBox::critical(QApplication::activeWindow(), "Error", err);
        return -1;
    }

    client_pid = process->processId();

    return app.exec();
}
