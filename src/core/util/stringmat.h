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

#include <vector>
#include <string>
#include "traced_assert.h"

#include "json_fwd.hpp"

class QAbstractItemModel;

namespace Utils
{

/**
*/
class StringMatrix
{
public:
    StringMatrix() = default;
    StringMatrix(const std::vector<std::string>& data, 
                 size_t rows, 
                 size_t cols);
    StringMatrix(const std::vector<std::vector<std::string>>& data);
    virtual ~StringMatrix() = default;

    size_t numRows() const { return nrows_; }
    size_t numCols() const { return ncols_; }

    void clear();

    std::string* rowPtr(size_t row);
    const std::string* rowPtr(size_t row) const;

    std::string& operator () (size_t row, size_t col);
    const std::string& operator () (size_t row, size_t col) const;

    void setData(const std::vector<std::string>& data, 
                 size_t rows, 
                 size_t cols);
    void setData(const std::vector<std::vector<std::string>>& data);

    void resize(size_t rows, size_t cols);

    nlohmann::json toJSON(bool rowwise = true,
                          const std::vector<int>& cols = std::vector<int>()) const;
    bool fromJSON(const nlohmann::json& obj);

protected:
    size_t index(size_t row, size_t col) const;

    std::vector<std::string> data_;
    size_t                   nrows_ = 0;
    size_t                   ncols_ = 0;
    size_t                   n_     = 0;
};

/**
*/
class StringTable
{
public:
    StringTable() = default;
    StringTable(const std::vector<std::string>& header,
                const std::vector<std::string>& data, 
                size_t rows, 
                size_t cols);
    StringTable(const std::vector<std::string>& header,
                const std::vector<std::vector<std::string>>& data);
    StringTable(const QAbstractItemModel* model);
    virtual ~StringTable() = default;

    void clear();

    void setData(const std::vector<std::string>& header,
                 const std::vector<std::string>& data, 
                 size_t rows, 
                 size_t cols);
    void setData(const std::vector<std::string>& header,
                 const std::vector<std::vector<std::string>>& data);
    void setData(const QAbstractItemModel* model);

    nlohmann::json toJSON(bool rowwise = true,
                          const std::vector<int>& cols = std::vector<int>()) const;
    bool fromJSON(const nlohmann::json& obj);

private:
    StringMatrix             data_;
    std::vector<std::string> header_;
};

}  // namespace Utils
