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

#include "logstream.h"

#include "json.hpp"
#include "property.h"
#include "propertylist.h"

#include <string>
#include <sstream>
#include <functional>

#include <boost/optional.hpp>

#include <QDateTime>
#include <QIcon>
#include <QAbstractItemModel>

enum class LogStreamType { Info, Warning, Error };

class LogStore : public QAbstractItemModel
{
    Q_OBJECT

signals:
    void messagesChangedSignal();

public slots:
    void databaseOpenedSlot();
    void databaseClosedSlot();

public:
    struct LogEntry {
        LogEntry(unsigned int msg_id, bool accepted, const std::string timestamp, LogStreamType type,
                 const std::string& component,
                 const std::string& message, boost::optional<unsigned int> error_code, const nlohmann::json& json)
            : msg_id_(msg_id),accepted_(accepted),
            timestamp_(timestamp),
            type_(type),
            component_(component),
            message_(message),
            error_code_(error_code),
            json_(json)
        {
        }

        LogEntry(const nlohmann::json& info); // from database

        const unsigned int msg_id_;
        bool accepted_;
        std::string timestamp_;
        LogStreamType type_;
        std::string component_; // A, A:B
        std::string message_;
        boost::optional<unsigned int> error_code_;
        nlohmann::json json_;
        unsigned int message_count_ {0};

        nlohmann::json asJSON() const;

        static std::string logStreamTypeStr (LogStreamType type);
        static LogStreamType logStreamTypeFromStr (const std::string& type_str);

        static const Property     DBColumnID;
        static const Property     DBColumnInfo;
        static const PropertyList DBPropertyList;
    };

    LogStore(bool show_everything);
    virtual ~LogStore() = default;

    LogStream logInfo(const std::string& component,
                      boost::optional<unsigned int> error_code, nlohmann::json json_blob);
    LogStream logWarn(const std::string& component,
                      boost::optional<unsigned int> error_code, nlohmann::json json_blob);
    LogStream logError(const std::string& component,
                       boost::optional<unsigned int> error_code, nlohmann::json json_blob);

    void addLogMessage(const std::string& message, LogStreamType type, const std::string& component,
                       boost::optional<unsigned int> error_code, nlohmann::json json_blob);

    void acceptMessages();

    const std::vector<LogEntry>& logEntries() const { return log_entries_; }

    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    bool setData(const QModelIndex &index, const QVariant& value, int role) override;

    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex& index) const override;

    Qt::ItemFlags flags(const QModelIndex &index) const override;

    void clearMessages();
    void loadMessagesFromDB();

protected:
    QStringList table_columns_;

    std::vector<LogEntry> log_entries_;

    QIcon checked_icon_;
};
