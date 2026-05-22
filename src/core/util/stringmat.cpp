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

#include "stringmat.h"
#include "json.hpp"

#include <set>

#include <QAbstractItemModel>

namespace Utils
{

/****************************************************************************************
 * StringMatrix
 ****************************************************************************************/

/**
*/
StringMatrix::StringMatrix(const std::vector<std::string>& data, 
                           size_t rows,
                           size_t cols)
{
    setData(data, rows, cols);
}

/**
*/
StringMatrix::StringMatrix(const std::vector<std::vector<std::string>>& data)
{
    setData(data);
}

/**
*/
void StringMatrix::clear()
{
    data_.clear();

    nrows_ = 0;
    ncols_ = 0;
    n_     = 0;
}

/**
*/
std::string* StringMatrix::rowPtr(size_t row)
{
    traced_assert(row < nrows_);
    return data_.data() + row * ncols_;
}

/**
*/
const std::string* StringMatrix::rowPtr(size_t row) const
{
    traced_assert(row < nrows_);
    return data_.data() + row * ncols_;
}

/**
*/
std::string& StringMatrix::operator () (size_t row, size_t col)
{
    return data_[ index(row, col) ];
}

/**
*/
const std::string& StringMatrix::operator () (size_t row, size_t col) const
{
    return data_[ index(row, col) ];
}

/**
*/
void StringMatrix::setData(const std::vector<std::string>& data, 
                           size_t rows, 
                           size_t cols)
{
    data_  = data;
    nrows_ = rows;
    ncols_ = cols;
    n_     = nrows_ * ncols_;
}

/**
*/
void StringMatrix::setData(const std::vector<std::vector<std::string>>& data)
{
    clear();

    nrows_ = data.size();
    ncols_ = data.empty() ? 0 : data[ 0 ].size();
    n_     = nrows_ * ncols_;

    for (size_t row = 0; row < nrows_; ++row)
    {
        traced_assert(data[ row ].size() == ncols_);
        data_.insert(data_.end(), data[ row ].begin(), data[ row ].end());
    }
}

/**
*/
void StringMatrix::resize(size_t rows, size_t cols)
{
    clear();

    if (rows == 0 || cols == 0)
        return;

    nrows_ = rows;
    ncols_ = cols;
    n_     = rows * cols;
    
    data_.resize(n_);
}

/**
*/
nlohmann::json StringMatrix::toJSON(bool rowwise,
                                    const std::vector<int>& cols) const
{
    std::vector<char> export_col(ncols_, cols.empty() ? 1 : 0);

    for (int idx : cols)
        export_col.at(idx) = 1;

    nlohmann::json json_data = nlohmann::json::array();

    if (rowwise)
    {
        for (size_t row = 0; row < nrows_; ++row)
        {
            nlohmann::json table_row = nlohmann::json::array();

            auto row_ptr = rowPtr(row);

            for (size_t col = 0; col < ncols_; ++col)
                if (export_col[ col ])
                    table_row.push_back(row_ptr[ col ]);
            
            json_data.push_back(table_row);
        }
    }
    else
    {
        std::vector<size_t> offsets(nrows_, 0);
        for (size_t r = 1; r < nrows_; ++r)
            offsets[ r ] = offsets[ r - 1 ] + ncols_;

        for (size_t col = 0; col < ncols_; ++col)
        {
            if (export_col[ col ] == 0)
                continue;

            nlohmann::json table_col = nlohmann::json::array();

            for (size_t row = 0; row < nrows_; ++row)
                table_col.push_back(data_[ offsets[ row ] + col ]);

            json_data.push_back(table_col);
        }
    }

    return json_data;
}

/**
*/
bool StringMatrix::fromJSON(const nlohmann::json& obj)
{
    clear();

    if (!obj.is_array())
        return false;

    size_t nrows = 0;
    size_t ncols = 0;
    std::vector<std::string> data;

    nrows = obj.size();

    bool   first = true;
    size_t cnt   = 0;

    for (size_t r = 0; r < nrows; ++r)
    {
        const auto& json_row = obj[ r ];
        if (!json_row.is_array())
            return false;

        if (first)
        {
            ncols = json_row.size();
            data.resize(ncols * nrows);
            first = false;
        }

        if (json_row.size() != ncols)
            return false;

        for (size_t c = 0; c < ncols; ++c)
            data[ cnt++ ] = json_row[ c ];
    }

    setData(data, nrows, ncols);

    return true;
}

/**
*/
size_t StringMatrix::index(size_t row, size_t col) const
{
    traced_assert(row < nrows_ && col < ncols_);
    return row * ncols_ + col;
}

/****************************************************************************************
 * StringTable
 ****************************************************************************************/

/**
*/
StringTable::StringTable(const std::vector<std::string>& header,
                         const std::vector<std::string>& data, 
                         size_t rows, 
                         size_t cols)
{
    setData(header, data, rows, cols);
}

/**
*/
StringTable::StringTable(const std::vector<std::string>& header,
                         const std::vector<std::vector<std::string>>& data)
{
    setData(header, data);
}

/**
*/
StringTable::StringTable(const QAbstractItemModel* model)
{
    setData(model);
}

/**
*/
void StringTable::clear()
{
    data_.clear();
    header_.clear();
}

/**
*/
void StringTable::setData(const std::vector<std::string>& header,
                          const std::vector<std::string>& data, 
                          size_t rows, 
                          size_t cols)
{
    traced_assert(header.size() == cols);

    data_.setData(data, rows, cols);

    header_ = header;
};

/**
*/
void StringTable::setData(const std::vector<std::string>& header,
                          const std::vector<std::vector<std::string>>& data)
{
    data_.setData(data);

    traced_assert(header_.size() == data_.numCols());

    header_ = header;
}

/**
*/
void StringTable::setData(const QAbstractItemModel* model)
{
    clear();

    if (!model)
        return;

    int nr = model->rowCount();
    int nc = model->columnCount();

    size_t nrows = std::max(0, nr);
    size_t ncols = std::max(0, nc);
    size_t n     = nrows * ncols;

    std::vector<std::string> table_header(ncols);
    std::vector<std::string> table_data(n);

    for (int col = 0; col < nc; ++col)
        table_header[ col ] = model->headerData(col, Qt::Horizontal, Qt::DisplayRole).toString().toStdString();

    size_t cnt = 0;
    for (int row = 0; row < nr; ++row)
        for (int col = 0; col < nc; ++col)
            table_data[ cnt++ ] = model->data(model->index(row, col), Qt::DisplayRole).toString().toStdString();

    setData(table_header, table_data, nrows, ncols);
};

/**
*/
nlohmann::json StringTable::toJSON(bool rowwise,
                                   const std::vector<int>& cols) const
{
    auto json_data = data_.toJSON(rowwise, cols);

    size_t nc = header_.size();

    std::set<int> col_set(cols.begin(), cols.end());

    nlohmann::json json_header = nlohmann::json::array();
    for (size_t col = 0; col < nc; ++col)
        if (col_set.empty() || col_set.count(col) > 0)
            json_header.push_back(header_[ col ]);

    nlohmann::json json_obj = nlohmann::json::object();
    json_obj[ "header" ] = json_header;
    json_obj[ "data"   ] = json_data;

    return json_obj;
}

/**
*/
bool StringTable::fromJSON(const nlohmann::json& obj)
{
    clear();

    if (!obj.is_object() || !obj.contains("header") || !obj.contains("data"))
        return false;

    if (!data_.fromJSON(obj[ "data" ]))
        return false;

    const auto& json_header = obj[ "header" ];
    if (!json_header.is_array())
        return false;

    size_t nc = json_header.size();
    if (nc != data_.numCols())
        return false;

    header_.resize(nc);

    for (size_t c = 0; c < nc; ++c)
        header_[ c ] = json_header[ c ];

    return true;
}

}  // namespace Utils
