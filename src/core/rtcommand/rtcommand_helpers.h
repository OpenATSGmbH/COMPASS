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

#include "rtcommand_defs.h"

#include <vector>
#include <string>

#include <QStringList>

#include <boost/optional.hpp>

class Configurable;

class QMainWindow;
class QDialog;
class QObject;
class QWidget;

namespace rtcommand
{

boost::optional<std::vector<std::string>> validObjectPath(const std::string& path);
boost::optional<std::pair<std::string, std::string>> signalFromObjectPath(const std::string& path);

QMainWindow* mainWindow();
QDialog* activeDialog();
std::pair<FindObjectErrCode, QObject*> getCommandReceiver(const std::string& object_path); // mainwindow.geographicview1, dialog.obj2, compass.child1

std::vector<std::string> parameterToStrings(const std::string& string_param);
std::string parameterFromStrings(const std::vector<std::string>& strings);
QStringList parameterToStrings(const QString& string_param);
QString parameterFromStrings(const QStringList& strings);

boost::optional<std::vector<int>> parameterToIntVector(const std::string& string_param);
boost::optional<std::vector<int>> parameterToIntVector(const QString& string_param);

QString getObjectPath(const QObject* obj);

QString getTooltip(const QObject* rtcmd_object = nullptr,
                   const Configurable* rtcmd_configurable = nullptr);

} // rtcommand
