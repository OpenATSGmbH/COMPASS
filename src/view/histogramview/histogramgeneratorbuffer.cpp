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

#include "histogramgeneratorbuffer.h"
#include "logger.h"
#include "variable.h"
#include "metavariable.h"

#include <QApplication>

/**
 */
HistogramGeneratorBuffer::HistogramGeneratorBuffer(Data* buffer_data, 
                                                   dbContent::Variable* variable,
                                                   dbContent::MetaVariable* meta_variable)
:   buffer_data_  (buffer_data)
,   variable_     (variable)
,   meta_variable_(meta_variable)
{
}

/**
 */
bool HistogramGeneratorBuffer::hasData() const
{
    return (buffer_data_ && (variable_ || meta_variable_));
}

/**
 */
dbContent::Variable* HistogramGeneratorBuffer::currentVariable(const std::string& db_content) const
{
    dbContent::Variable* data_var = nullptr;

    if (meta_variable_)
    {
        if (!meta_variable_->existsIn(db_content))
        {
            logwrn << "meta var does not exist in dbcontent '" << db_content << "'";
            return nullptr;
        }
        data_var = &meta_variable_->getFor(db_content);
    }
    else
    {
        data_var = variable_;

        if (data_var && data_var->dbContentName() != db_content)
            return nullptr;
    }

    return data_var;
}

/**
 */
bool HistogramGeneratorBuffer::select_impl(unsigned int bin0, unsigned int bin1)
{
    if (!hasData())
        return false;

    //determine some options
    auto num_bins = currentBins();

    bool select_null    = bin1 == num_bins;
    bool select_min_max = !(bin0 == bin1 && bin1 == num_bins); // not b´oth num bins

    if (select_null)
        bin1 -= 1;

    bool add_to_selection = QApplication::keyboardModifiers() & Qt::ControlModifier;

    bool ok = true;

    //run selection on all buffers
    for (auto& elem : *buffer_data_)
    {
        ok = ok && selectBuffer(elem.first, *elem.second, bin0, bin1, select_min_max, select_null, add_to_selection);
    }

    return ok;
}
