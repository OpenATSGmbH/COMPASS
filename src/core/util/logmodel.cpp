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

#include "logmodel.h"
#include "logger.h"
#include "stringconv.h"
#include "files.h"
#include "compass.h"
#include "dbinterface.h"

#include <QBrush>
#include <QFont>

using namespace Utils;

// bool accepted;
// QString timestamp;
// LogStreamType type;
// std::string component_; // A, A:B
// std::string message;
// boost::optional<unsigned int> error_code_;
// nlohmann::json json_;
// unsigned int message_count_ {0};

const std::string LOGENTRY_MSG_ID_KEY {"msg_id"};
const std::string LOGENTRY_ACCEPTED_KEY {"accepted"};
const std::string LOGENTRY_TIMESTAMP_KEY {"timestamp"};
const std::string LOGENTRY_TYPE_STR_KEY {"type_str"};
const std::string LOGENTRY_COMPONENT_KEY {"component"};
const std::string LOGENTRY_MESSAGE_KEY {"message"};
const std::string LOGENTRY_ERROR_CODE_KEY {"error_code"};
const std::string LOGENTRY_JSON_KEY {"json"};
const std::string LOGENTRY_MESSAGE_COUNT_KEY {"message_count"};

const Property     LogStore::LogEntry::DBColumnID     = Property("msg_id" , PropertyDataType::UINT);
const Property     LogStore::LogEntry::DBColumnInfo   = Property("json", PropertyDataType::JSON);
const PropertyList LogStore::LogEntry::DBPropertyList = PropertyList({ LogStore::LogEntry::DBColumnID,
                                                          LogStore::LogEntry::DBColumnInfo });

LogStore::LogEntry::LogEntry(const nlohmann::json& info)
    : msg_id_(info.at(LOGENTRY_MSG_ID_KEY))
{
    traced_assert(info.contains(LOGENTRY_ACCEPTED_KEY));
    accepted_ = info.at(LOGENTRY_ACCEPTED_KEY);

    traced_assert(info.contains(LOGENTRY_TIMESTAMP_KEY));
    timestamp_ = info.at(LOGENTRY_TIMESTAMP_KEY);

    traced_assert(info.contains(LOGENTRY_TYPE_STR_KEY));
    type_ = logStreamTypeFromStr(info.at(LOGENTRY_TYPE_STR_KEY));

    traced_assert(info.contains(LOGENTRY_COMPONENT_KEY));
    component_ = info.at(LOGENTRY_COMPONENT_KEY);

    traced_assert(info.contains(LOGENTRY_MESSAGE_KEY));
    message_ = info.at(LOGENTRY_MESSAGE_KEY);

    if (info.contains(LOGENTRY_ERROR_CODE_KEY))
        error_code_ = info.at(LOGENTRY_ERROR_CODE_KEY);

    traced_assert(info.contains(LOGENTRY_JSON_KEY));
    json_ = info.at(LOGENTRY_JSON_KEY);

    traced_assert(info.contains(LOGENTRY_MESSAGE_COUNT_KEY));
    message_count_ = info.at(LOGENTRY_MESSAGE_COUNT_KEY);
}


nlohmann::json LogStore::LogEntry::asJSON() const
{
    nlohmann::json info;

    info[LOGENTRY_MSG_ID_KEY] = msg_id_;

    info[LOGENTRY_ACCEPTED_KEY] = accepted_;

    info[LOGENTRY_TIMESTAMP_KEY] = timestamp_;

    info[LOGENTRY_TYPE_STR_KEY] = logStreamTypeStr(type_);

    info[LOGENTRY_COMPONENT_KEY] = component_;
    info[LOGENTRY_MESSAGE_KEY] = message_;

    if (error_code_)
        info[LOGENTRY_ERROR_CODE_KEY] = *error_code_;

    info[LOGENTRY_JSON_KEY] = json_;

    info[LOGENTRY_MESSAGE_COUNT_KEY] = message_count_;

    return info;
}


LogStore::LogStore(bool show_everything)
{
    table_columns_ = QStringList::fromVector({"", "Time", "Type", "Component", "Message"});

    if (show_everything)
        table_columns_.append(QStringList::fromVector({"Error", "JSON", "Count"}));

    checked_icon_ = QIcon(Files::getIconFilepath("done.png").c_str());
}

LogStream LogStore::logInfo(const std::string& component,
                            boost::optional<unsigned int> error_code, nlohmann::json json_blob) {
    return LogStream([this, component, error_code, json_blob](const std::string& msg) {
        addLogMessage(msg, LogStreamType::Info, component, error_code, json_blob); });
}
LogStream LogStore::logWarn(const std::string& component,
                            boost::optional<unsigned int> error_code, nlohmann::json json_blob) {

    LogStream::CommitFunc func = [this, component, error_code, json_blob](const std::string& msg) {
        addLogMessage(msg, LogStreamType::Warning, component, error_code, json_blob); };

    return LogStream(func);
}
LogStream LogStore::logError(const std::string& component,
                             boost::optional<unsigned int> error_code, nlohmann::json json_blob) {

    LogStream::CommitFunc func = [this, component, error_code, json_blob](const std::string& msg) {
        addLogMessage(msg, LogStreamType::Error, component, error_code, json_blob); };

    return LogStream(func);
}

void LogStore::addLogMessage(const std::string& message, LogStreamType type, const std::string& component,
                             boost::optional<unsigned int> error_code, nlohmann::json json_blob)
{
    beginResetModel();

    log_entries_.push_back(
        LogEntry(log_entries_.size(), false, QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss").toStdString(), type,
                 component, message, error_code, json_blob));

    const LogEntry& entry = *log_entries_.rbegin();

    COMPASS::instance().dbInterface().saveTaskLogInfo(entry.msg_id_, entry.asJSON());

    endResetModel();

    emit messagesChangedSignal();
}

void LogStore::acceptMessages()
{
    beginResetModel();

    for (auto& entry : log_entries_)
        entry.accepted_ = true;

    endResetModel();

    emit messagesChangedSignal();
}

QVariant LogStore::data(const QModelIndex& index, int role) const
{
    if (!index.isValid())
        return QVariant();

    traced_assert(index.row() >= 0);
    traced_assert((unsigned int)index.row() < log_entries_.size());

    const LogEntry& entry = log_entries_.at(index.row());

    switch (role)
    {
    // case Qt::CheckStateRole:
    // {
    //     if (index.column() == 0)  // selected special case
    //     {
    //         if (entry.accepted)
    //             return Qt::Checked;
    //         else
    //             return Qt::Unchecked;
    //     }
    //     else
    //         return QVariant();
    // }
    case Qt::ForegroundRole:
    {
        if (!entry.accepted_)
        {
            switch(entry.type_)
            {
            case LogStreamType::Error:
                return QBrush(Qt::red);
            case LogStreamType::Warning:
                return QBrush(QColor("orange"));
            case LogStreamType::Info:
            default:
                break;
            }
        }

        return QVariant();
    }
    case Qt::BackgroundRole:
    {
        if (entry.accepted_)
            return QBrush(QColor("gainsboro"));
        else
            return QVariant();

    }
    case Qt::FontRole:
    {
        QFont font;

        if (!entry.accepted_)
        {
            switch(entry.type_)
            {
            case LogStreamType::Error:
                font.setBold(true);
                break;
            case LogStreamType::Warning:
                font.setItalic(true);
                break;
            case LogStreamType::Info:
            default:
                break;
            }
        }

        return font;
    }
    case Qt::DisplayRole:
    //case Qt::EditRole:
    {
        logdbg << "display role: row " << index.row() << " col " << index.column();

        traced_assert(index.column() < table_columns_.size());
        std::string col_name = table_columns_.at(index.column()).toStdString();

        if (col_name == "")
        {
            return QVariant();
        }
        else if (col_name == "Time")
        {
            return entry.timestamp_.c_str();
        }
        else if (col_name == "Type")
        {
            switch(entry.type_)
            {
            case LogStreamType::Info:
                return "Info";
            case LogStreamType::Warning:
                return "Warning";
            case LogStreamType::Error:
                return "Error";
            default:
                return "Unknown";;
            }
        }
        else if (col_name == "Component")
        {
            return entry.component_.c_str();
        }
        else if (col_name == "Message")
        {
            return entry.message_.c_str();
        }
        else if (col_name == "Error")
        {
            return entry.error_code_ ?
                       ("0x"+String::hexStringFromInt((unsigned int) *entry.error_code_)).c_str() : QVariant();
        }
        else if (col_name == "JSON")
        {
            return entry.json_.empty() ? QVariant() : entry.json_.dump(2).c_str();
        }
        else if (col_name == "Count")
        {
            return entry.message_count_ ? entry.message_count_ : QVariant();
        }

    }
    case Qt::DecorationRole:
    {
        if (index.column() > 0)  // only col 0 have icons
            return QVariant();

        if (entry.accepted_)
            return checked_icon_;
        else
            return QVariant();;
    }
    case Qt::TextAlignmentRole:
    {
        return (int)(Qt::AlignLeft | Qt::AlignTop);
    }
    // case Qt::UserRole: // to find the checkboxes
    // {
    //     if (index.column() == 0)
    //     {
    //         traced_assert(index.row() >= 0);
    //         traced_assert(index.row() < target_data_.size());

    //         const Target& target = target_data_.at(index.row());
    //         return target.utn_;
    //     }
    //     else if (index.column() == 2) // comment
    //     {
    //         traced_assert(index.row() >= 0);
    //         traced_assert(index.row() < target_data_.size());

    //         const Target& target = target_data_.at(index.row());
    //         return ("comment_"+to_string(target.utn_)).c_str();
    //     }
    // }
    default:
    {
        return QVariant();
    }
    }
}

bool LogStore::setData(const QModelIndex &index, const QVariant& value, int role)
{
    // if (!index.isValid() /*|| role != Qt::EditRole*/)
    //     return false;

    // if (role == Qt::CheckStateRole && index.column() == 0)
    // {
    //     traced_assert(index.row() >= 0);
    //     traced_assert(index.row() < target_data_.size());

    //     auto it = target_data_.begin()+index.row();

    //     bool checked = (Qt::CheckState)value.toInt() == Qt::Checked;
    //     loginf << "utn " << it->utn_ <<" check state " << checked;

    //     //eval_man_.useUTN(it->utn_, checked, false);
    //     target_data_.modify(it, [value,checked](Target& p) { p.useInEval(checked); });

    //     saveToDB(it->utn_);

    //     emit dataChanged(index, LogStore::index(index.row(), columnCount()-1));
    //     emit dbcont_manager_.targetChangedSignal(it->utn_);

    //     return true;
    // }
    // else if (role == Qt::EditRole && index.column() == 2) // comment
    // {
    //     traced_assert(index.row() >= 0);
    //     traced_assert(index.row() < target_data_.size());

    //     auto it = target_data_.begin()+index.row();

    //     loginf << "utn " << it->utn_ <<" comment '" << value.toString().toStdString() << "'";

    //     target_data_.modify(it, [value](Target& p) { p.comment(value.toString().toStdString()); });

    //     saveToDB(it->utn_);

    //     emit dbcont_manager_.targetChangedSignal(it->utn_);

    //     return true;
    // }

    return false;
}


QVariant LogStore::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
    {
        traced_assert(section < table_columns_.size());
        return table_columns_.at(section);
    }

    return QVariant();
}

QModelIndex LogStore::index(int row, int column, const QModelIndex& parent) const
{
    return createIndex(row, column);
}

int LogStore::rowCount(const QModelIndex& parent) const
{
    return log_entries_.size();
}

int LogStore::columnCount(const QModelIndex& parent) const
{
    return table_columns_.size();
}

QModelIndex LogStore::parent(const QModelIndex& index) const
{
    return QModelIndex();
}

Qt::ItemFlags LogStore::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return Qt::ItemIsEnabled;

    traced_assert(index.column() < table_columns_.size());

    // if (index.column() == 0) // Use
    // {
    //     return QAbstractItemModel::flags(index) | Qt::ItemIsUserCheckable;
    // }
    // else if (index.column() == 2) // comment
    // {
    //     return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
    // }
    // else
        return QAbstractItemModel::flags(index);
}

std::string LogStore::LogEntry::logStreamTypeStr (LogStreamType type)
{
    // enum class LogStreamType { Info, Warning, Error };

    switch(type)
    {
    case LogStreamType::Info:
        return "Info";
    case LogStreamType::Warning:
        return "Warning";
    case LogStreamType::Error:
        return "Error";
    default:
        traced_assert(false);
    }

}

LogStreamType LogStore::LogEntry::logStreamTypeFromStr (const std::string& type_str)
{
    if (type_str == "Info")
        return LogStreamType::Info;
    if (type_str == "Warning")
        return LogStreamType::Warning;
    if (type_str == "Error")
        return LogStreamType::Error;
    else
        traced_assert(false);
}

void LogStore::clearMessages()
{
    loginf;

    beginResetModel();

    log_entries_.clear();

    endResetModel();

}
void LogStore::loadMessagesFromDB()
{
    loginf;

    beginResetModel();

    log_entries_.clear();

    for (auto& info : COMPASS::instance().dbInterface().loadTaskLogInfo())
        log_entries_.emplace_back(info);

    endResetModel();

    emit messagesChangedSignal();
}

void LogStore::databaseOpenedSlot()
{
    loadMessagesFromDB();
}
void LogStore::databaseClosedSlot()
{
    clearMessages();
}
