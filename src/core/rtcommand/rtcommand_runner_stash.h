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

#pragma once

#include <memory>

#include <QObject>

class COMPASS;

class QSignalSpy;

namespace rtcommand
{
    struct RTCommand;
}

/**
*/
struct RTCommandMetaTypeWrapper
{
    std::shared_ptr<rtcommand::RTCommand> command;
};

namespace rtcommand
{

/**
 * Obtains data structures and calls for the command runner 
 * needed to reside in the main thread.
 */
class RTCommandRunnerStash : public QObject
{
    Q_OBJECT
public:
    RTCommandRunnerStash(COMPASS& compass);
    virtual ~RTCommandRunnerStash();

private slots:
    bool spyForSignal(const QString& obj_name, const QString& signal_name);
    void removeSpy();
    bool executeCommand(RTCommandMetaTypeWrapper wrapper) const;
    void executeCommandAsync(RTCommandMetaTypeWrapper wrapper) const;
    bool postCheckCommand(RTCommandMetaTypeWrapper wrapper) const;
    // slot: must run in the main thread - the QSignalSpy list is appended there
    // on signal delivery, reading it from the runner thread is a data race
    bool spySignalReceived() const;

private:
    friend class RTCommandRunner;

    COMPASS& compass_;
    std::unique_ptr<QSignalSpy> spy_;
};

} // namespace rtcommand

Q_DECLARE_METATYPE(RTCommandMetaTypeWrapper)
