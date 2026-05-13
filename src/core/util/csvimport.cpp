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

#include "csvimport.h"

#include "logger.h"

#include <string>

#include <QStringList>
#include <QFile>
#include <QTextStream>

namespace csv
{

/**
 */
bool CSVImport::parseCsvLine(QStringList& fields, const QString& line, const QChar& sep) const
{
    fields.clear();

    QString field;
    size_t num_quotes = 0;

    for (int i = 0; i < line.size(); ++i)
    {
        QChar c = line[i];

        bool in_quotes = num_quotes % 2 == 1;

        if (c == '"')
        {
            //count quotes
            ++num_quotes;
        }
        else if (c == sep && !in_quotes)
        {
            //collect field at separator
            fields << field;
            field.clear();
        }
        else
        {
            //add to field content
            field.append(c);
        }
    }

    fields << field;

    //quote count must be even
    bool ok = num_quotes % 2 == 0;

    if (!ok)
    {
        logerr << "parseCsvLine failed: odd quote count " << num_quotes
               << " line '" << line.toStdString() << "'";
        for (int i = 0; i < fields.size(); ++i)
            logerr << "parseCsvLine failed: field[" << i << "] '" << fields[i].toStdString() << "'";
    }

    return ok;
}

/**
 */
bool CSVImport::parseDecimalCommaNumber(const QString& field, double& number, bool decimal_comma) const
{
    auto f = field;
    if (decimal_comma && f.count(','))
        f.replace(',', '.');

    bool ok = false;
    number = field.toDouble(&ok);

    return ok;
}

/**
 */
ResultT<nlohmann::json> CSVImport::parse(const std::string& fn,
                                         const QChar& separator,
                                         bool decimal_comma)
{
    auto lines = nlohmann::json::array();

    try
    {
        QFile file(QString::fromStdString(fn));
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
            return ResultT<nlohmann::json>::failed("could not open file");

        QString line;
        QStringList fields;
        boost::optional<size_t> num_cols;

        QTextStream in(&file);
        while (!in.atEnd()) 
        {
            line = in.readLine();
            if (in.status() != QTextStream::Status::Ok)
                return ResultT<nlohmann::json>::failed("invalid text stream");

            if (line.isEmpty())
                continue;

            if (!parseCsvLine(fields, line, separator))
            {
                logerr << "could not parse line " << (lines.size() + 1)
                       << " content '" << line.toStdString() << "'";
                return ResultT<nlohmann::json>::failed("could not parse line " + std::to_string(lines.size() + 1));
            }

            size_t n = (size_t)fields.size();

            if (!num_cols.has_value())
                num_cols = n;
            else if (num_cols.value() != n)
            {
                logerr << "unmatched field count in line " << (lines.size() + 1)
                       << " expected " << num_cols.value() << " got " << n
                       << " content '" << line.toStdString() << "'";
                return ResultT<nlohmann::json>::failed("unmatched field count in line " + std::to_string(lines.size() + 1));
            }

            auto j_fields = nlohmann::json::array();
            double num;
            for (const auto& field : fields)
            {
                if (parseDecimalCommaNumber(field, num, decimal_comma))
                    j_fields.push_back(num);
                else
                    j_fields.push_back(field.toStdString());
            }

            traced_assert(j_fields.size() == n);

            lines.push_back(j_fields);
        }
    }
    catch(const std::exception& e)
    {
        return ResultT<nlohmann::json>::failed(e.what());
    }
    catch(...)
    {
        return ResultT<nlohmann::json>::failed("unknown error");
    }

    return ResultT<nlohmann::json>::succeeded(lines);
}

} // namespace csv
