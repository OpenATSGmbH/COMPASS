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

#include "result.h"

#include <json.hpp>

#include <QChar>

class QString;
class QStringList;

namespace csv
{

class CSVImport
{
public:
    CSVImport() = default;
    virtual ~CSVImport() = default;

    ResultT<nlohmann::json> parse(const std::string& fn,
                                  const QChar& separator = ';',
                                  bool decimal_comma = false);
private:
    bool parseDecimalCommaNumber(const QString& field, double& number, bool decimal_comma) const;
    bool parseCsvLine(QStringList& fields, const QString &line, const QChar& sep) const;
};

} // namespace csv
