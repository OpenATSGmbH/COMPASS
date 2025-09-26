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

#include "global.h"
#include "buffer.h"
#include "property.h"
#include "stringconv.h"
#include "json.hpp"

#include <QDateTime>

#include <boost/optional.hpp>

#include <map>
#include <set>
#include <vector>
#include <type_traits>


const bool BUFFER_PEDANTIC_CHECKING = false;

template <class T>
class NullableVector
{
    friend class Buffer;

public:
    virtual ~NullableVector() {}

    void renameProperty(const std::string& name) { property_.rename(name); }

    void clear();
    void clearData(); // removes all data

    const T get(unsigned int index) const;

    //template<typename T_ = T, typename std::enable_if<!std::is_integral<T_>::value>::type* = nullptr>
    //T_& getRef(unsigned int index)
    T& getRef(unsigned int index)
    {
        logdbg2 << property_.name() << ": index " << index;
        if (BUFFER_PEDANTIC_CHECKING)
        {
            traced_assert(data_.size() <= buffer_.size_);
            traced_assert(null_flags_.size() <= buffer_.size_);
            traced_assert(index < data_.size());
        }

        if (isNull(index))
        {
            logerr << property_.name() << ": getRef: index " << index << " is null";
            traced_assert(false);
        }

        return data_.at(index);
    }

    const std::string getAsString(unsigned int index) const;

    void set(unsigned int index, T value);
    void setFromFormat(unsigned int index, const std::string& format, const std::string& value_str, bool debug=false);
    void setAll(T value);

    void append(unsigned int index, T value);
    void appendFromFormat(unsigned int index, const std::string& format,
                          const std::string& value_str);

    void setNull(unsigned int index);
    void setAllNull();

    NullableVector<T>& operator*=(double factor);

    std::set<T> distinctValues(unsigned int index = 0);
    std::map<T, unsigned int> distinctValuesWithCounts(unsigned int index = 0);
    std::tuple<bool,T,T> minMaxValues(unsigned int index = 0); // set, min, max
    std::tuple<bool,T,T> minMaxValuesSorted(unsigned int index = 0); // set, min, max

    std::map<boost::optional<T>, std::vector<unsigned int>> distinctValuesWithIndexes(unsigned int from_index,
                                                                     unsigned int to_index);
    std::map<boost::optional<T>, std::vector<unsigned int>> distinctValuesWithIndexes(
            const std::vector<unsigned int>& indexes);

    std::map<T, unsigned int> uniqueValuesWithIndexes();
    std::map<T, unsigned int> uniqueValuesWithIndexes(const std::set<T>& values); // get indexes of given values
    // std::vector<unsigned int> nullValueIndexes(unsigned int from_index, unsigned int to_index);
    // std::vector<unsigned int> nullValueIndexes(const std::vector<unsigned int>& indexes);

    void convertToStandardFormat(const std::string& from_format);

    unsigned int contentSize();

    /// @brief Checks if specific element is Null
    bool isNull(unsigned int index) const;
    bool isAlwaysNull() const;
    bool isNeverNull() const;

    void swapData (unsigned int index1, unsigned int index2);

    std::string propertyName() const
    {
        return property_.name();
    }

    std::string propertyID() const
    {
        return property_.name() + "(" + property_.dataTypeString() + ")";
    }

    std::vector<unsigned int> sortPermutation();
    void sortByPermutation(const std::vector<unsigned int>& perm);

    nlohmann::json asJSON(unsigned int max_size=0);

private:
    Property property_;
    Buffer& buffer_;

    std::vector<T> data_;

    std::vector<bool> null_flags_;

    void unsetNull(unsigned int index);

    void resizeDataTo(unsigned int size);
    void resizeNullTo(unsigned int size);
    void addData(NullableVector<T>& other);
    void copyData(NullableVector<T>& other);
    void cutToSize(unsigned int size);
    void cutUpToIndex(unsigned int index); // everything up to index is removed
    void removeIndexes(const std::vector<unsigned int>& indexes_to_remove); // must be sorted

    NullableVector(Property& property, Buffer& buffer);
};

template <class T>
NullableVector<T>::NullableVector(Property& property, Buffer& buffer)
    : property_(property), buffer_(buffer)
{
}

template <class T>
void NullableVector<T>::clear()
{
    logdbg2 << property_.name();
    std::fill(data_.begin(), data_.end(), T());
    std::fill(null_flags_.begin(), null_flags_.end(), true);
}

template <class T>
void NullableVector<T>::clearData() // removes all data
{
    data_.clear();
    null_flags_.clear();
}

template <class T>
const T NullableVector<T>::get(unsigned int index) const
{
    logdbg2 << property_.name() << ": index " << index;
    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(data_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
        traced_assert(index < data_.size());
    }

    if (isNull(index))
    {
        logerr << property_.name() << ": get: index " << index << " is null";
        traced_assert(false);
    }

    return data_.at(index);
}

template <class T>
const std::string NullableVector<T>::getAsString(unsigned int index) const
{
    logdbg2 << property_.name();
    return Utils::String::getValueString(get(index));
}

template <class T>
void NullableVector<T>::set(unsigned int index, T value)
{
    logdbg2 << property_.name() << ": index " << index << " value '"
           << value << "'";

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(data_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
    }

    if (index >= data_.size())  // allocate new stuff, fill all new with not null
    {
        if (index != data_.size())  // some where left out
            resizeNullTo(index + 1);

        resizeDataTo(index + 1);
    }

    if (BUFFER_PEDANTIC_CHECKING)
        traced_assert(index < data_.size());

    data_.at(index) = value;
    unsetNull(index);

    // logdbg2 << "size " << size_ << " max_size " << max_size_;
}

template <class T>
void NullableVector<T>::setFromFormat(unsigned int index, const std::string& format,
                                      const std::string& value_str, bool debug)
{
    logdbg2 << property_.name();
    T value{};

    if (format == "octal")
    {
        value = std::stoi(value_str, 0, 8);
    }
    else if (format == "hexadecimal")
    {
        value = std::stoi(value_str, 0, 16);
    }
    else if (format == "epoch_tod_ms")
    {
        QDateTime date_time;
        date_time.setMSecsSinceEpoch(std::stoul(value_str));
        value = Utils::String::timeFromString(date_time.toString("hh:mm:ss.zzz").toStdString());
    }
    else if (format == "epoch_tod_s")
    {
        QDateTime date_time;
        date_time.setMSecsSinceEpoch(1000 * std::stoul(value_str));
        value = Utils::String::timeFromString(date_time.toString("hh:mm:ss.zzz").toStdString());
    }
    else if (format == "bool")
    {
        if (value_str == "0" || value_str == "false")
            value = 'N';
        else if (value_str == "1"  || value_str == "true")
            value = 'Y';
    }
    else if (format == "bool_invert")
    {
        if (value_str == "1"  || value_str == "true")
            value = 'N';
        else if (value_str == "0" || value_str == "false")
            value = 'Y';
    }
    else
    {
        logerr << "unknown format '" << format << "'";
        traced_assert(false);
    }

    if (debug)
        loginf << "index " << index << " value_str '" << value_str
               << "' value '" << value << "'";

    set(index, value);
}

template <class T>
void NullableVector<T>::setAll(T value)
{
    unsigned int data_size = data_.size();

    for (unsigned int index=0; index < data_size; ++index)
    {
        data_.at(index) = value;
        unsetNull(index);
    }
}

template <class T>
void NullableVector<T>::append(unsigned int index, T value)
{
    logdbg2 << property_.name() << ": index " << index << " value '"
           << value << "'";

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(data_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
    }

    if (index >= data_.size())  // allocate new stuff, fill all new with not null
    {
        if (index != data_.size())  // some where left out
            resizeNullTo(index + 1);

        resizeDataTo(index + 1);
    }

    if (BUFFER_PEDANTIC_CHECKING)
        traced_assert(index < data_.size());

    data_.at(index) += value;
    unsetNull(index);

    // logdbg2 << "NullableVector: size " << size_ << " max_size " << max_size_;
}

template <class T>
void NullableVector<T>::appendFromFormat(unsigned int index, const std::string& format,
                                         const std::string& value_str)
{
    logdbg2 << property_.name();
    T value;

    if (format == "octal")
    {
        value = std::stoi(value_str, 0, 8);
    }
    else if (format == "hexadecimal")
    {
        value = std::stoi(value_str, 0, 16);
    }
    else if (format == "epoch_tod_ms")
    {
        QDateTime date_time;
        date_time.setMSecsSinceEpoch(std::stoul(value_str));
        value = Utils::String::timeFromString(date_time.toString("hh:mm:ss.zzz").toStdString());
    }
    else if (format == "epoch_tod_s")
    {
        QDateTime date_time;
        date_time.setMSecsSinceEpoch(1000 * std::stoul(value_str));
        value = Utils::String::timeFromString(date_time.toString("hh:mm:ss.zzz").toStdString());
    }
    else
    {
        logerr << "unknown format '" << format << "'";
        traced_assert(false);
    }

    append(index, value);
}

template <class T>
void NullableVector<T>::setNull(unsigned int index)
{
    logdbg2 << property_.name() << ": index " << index;

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(data_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
    }

    if (index >= null_flags_.size())  // null flags to small
        resizeNullTo(index + 1);

    if (BUFFER_PEDANTIC_CHECKING)
        traced_assert(index < null_flags_.size());

    null_flags_.at(index) = true;
}

template <class T>
void NullableVector<T>::setAllNull()
{
    unsigned int data_size = data_.size();

    for (unsigned int cnt=0; cnt < data_size; ++cnt)
        setNull(cnt);
}


/// @brief Checks if specific element is Null
template <class T>
bool NullableVector<T>::isNull(unsigned int index) const
{
    logdbg2 << property_.name() << ": index " << index;

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(data_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
        traced_assert(index < buffer_.size_);
    }

    if (index < null_flags_.size())  // if stored, return value
        return null_flags_.at(index);

    // null not stored, so all set are not null

    if (index >= data_.size())  // not yet set
        return true;

    // must be set
    return false;
}

template <class T>
void NullableVector<T>::resizeDataTo(unsigned int size)
{
    logdbg2 << property_.name() << ": size " << size;

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(data_.size() <= buffer_.size_);
        traced_assert(data_.size() < size);  // only to be called if needed
    }

    data_.resize(size, T());

    if (buffer_.size_ < data_.size())  // set new data size
        buffer_.size_ = data_.size();
}

template <class T>
void NullableVector<T>::resizeNullTo(unsigned int size)
{
    logdbg2 << property_.name() << ": size " << size;

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(size >= null_flags_.size());
        traced_assert(null_flags_.size() <= buffer_.size_);
    }

    if (data_.size() > null_flags_.size())  // data was set w/o null, adjust & fill with set values
        null_flags_.resize(data_.size(), false);

    if (null_flags_.size() < size)  // adjust to new size, fill with null values
        null_flags_.resize(size, true);

    if (buffer_.size_ < null_flags_.size())  // set new data size
        buffer_.size_ = null_flags_.size();

    if (BUFFER_PEDANTIC_CHECKING)
        traced_assert(null_flags_.size() >= size); // could be larger since increase to data.size()
}

template <class T>
void NullableVector<T>::addData(NullableVector<T>& other)
{
    logdbg2 << property_.name();

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(data_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
    }

    if (!other.data_.size() &&
            other.null_flags_.size())  // if other has null flags set, need to fill my nulls
    {
        logdbg2 << property_.name()
               << ": 1: other no data resizing null";
        resizeNullTo(buffer_.size_);
        logdbg2 << property_.name() << ": 1: inserting null";
        null_flags_.insert(null_flags_.end(), other.null_flags_.begin(), other.null_flags_.end());
        return;
    }

    if (other.data_.size() && !other.null_flags_.size())  // if other has everything set
    {
        logdbg2 << property_.name()
               << ": 2: other has everything set";

        if (data_.size() < buffer_.size_)  // need to size data up
        {
            logdbg2 << property_.name()
               << ": 2: data not full, setting null";
            resizeNullTo(buffer_.size_);

            logdbg2 << property_.name() << ": 2: resizing data";
            resizeDataTo(buffer_.size_);
        }

        logdbg2 << property_.name() << ": 2: inserting data";
        data_.insert(data_.end(), other.data_.begin(), other.data_.end());
        return;
    }

    logdbg2 << property_.name() << ": 3: mixture, both have data & nulls";

    logdbg2 << property_.name() << ": 3: resizing null to "
           << buffer_.size_;
    resizeNullTo(buffer_.size_);
    logdbg2 << property_.name() << ": 3: inserting nulls";
    null_flags_.insert(null_flags_.end(), other.null_flags_.begin(), other.null_flags_.end());

    if (data_.size() < buffer_.size_)  // need to size data up
    {
        logdbg2 << property_.name() << ": 3: resizing data";
        resizeDataTo(buffer_.size_);
    }

    logdbg2 << property_.name() << ": 3: inserting data";
    data_.insert(data_.end(), other.data_.begin(), other.data_.end());

    // size is adjusted in Buffer::seizeBuffer
    logdbg2 << property_.name() << ": end";
}

template <class T>
void NullableVector<T>::copyData(NullableVector<T>& other)
{
    logdbg2 << property_.name();

    data_ = other.data_;
    null_flags_ = other.null_flags_;

    // is only done for new buffers in Buffer::getPartialCopy, so no size-too-big isse

    if (buffer_.size_ < data_.size())
        buffer_.size_ = data_.size();

    if (buffer_.size_ < null_flags_.size())
        buffer_.size_ = null_flags_.size();

    logdbg2 << property_.name() << ": end";
}

template <class T>
NullableVector<T>& NullableVector<T>::operator*=(double factor)
{
    logdbg2 << property_.name();

    unsigned int data_size = data_.size();

    //    tbb::parallel_for(uint(0), data_size, [&](unsigned int cnt) {
    //        if (!isNull(cnt))
    //        {
    //            data_.at(cnt) *= factor;
    //        }
    //    });

    for (unsigned int cnt=0; cnt < data_size; ++cnt)
    {
        if (!isNull(cnt))
            data_.at(cnt) *= factor;
    }

    //    for (auto &data_it : data_)
    //        data_it *= factor;

    return *this;
}

template <class T>
std::set<T> NullableVector<T>::distinctValues(unsigned int index)
{
    logdbg2 << property_.name();

    std::set<T> values;

    T value;

    for (; index < data_.size(); ++index)
    {
        if (!isNull(index))  // not for null
        {
            value = data_.at(index);
            if (values.count(value) == 0)
                values.insert(value);
        }
    }

    return values;
}

template <class T>
std::map<T, unsigned int> NullableVector<T>::distinctValuesWithCounts(unsigned int index)
{
    logdbg2 << property_.name();

    std::map<T, unsigned int> values;

    T value;

    for (; index < data_.size(); ++index)
    {
        if (!isNull(index))  // not for null
        {
            value = data_.at(index);
            values[value] += 1;
        }
    }

    return values;
}

template <class T>
std::tuple<bool,T,T> NullableVector<T>::minMaxValues(unsigned int index)
{
    bool set = false;
    T min{}, max{};

    for (; index < data_.size(); ++index)
    {
        if (!isNull(index))  // not for null
        {
            if (!set)
            {
                min = data_.at(index);
                max = data_.at(index);
                set = true;
            }
            else
            {
                min = std::min(min, data_.at(index));
                max = std::max(max, data_.at(index));
            }
        }
    }

    return std::tuple<bool,T,T> {set, min, max};
}

// template <>
// std::tuple<bool,boost::posix_time::ptime,boost::posix_time::ptime> NullableVector<boost::posix_time::ptime>::minMaxValues(unsigned int index)
// {
//     bool set = false;
//     boost::posix_time::ptime min{}, max{};

//     for (; index < data_.size(); ++index)
//     {
//         if (!isNull(index) && !data_.at(index).is_not_a_date_time())  // not for null
//         {
//             if (!set)
//             {
//                 min = data_.at(index);
//                 max = data_.at(index);
//                 set = true;
//             }
//             else
//             {
//                 min = std::min(min, data_.at(index));
//                 max = std::max(max, data_.at(index));
//             }
//         }
//     }

//     return std::tuple<bool,boost::posix_time::ptime,boost::posix_time::ptime> {set, min, max};
// }

/**
 * Special case for booleans.
 */
template <>
inline std::tuple<bool,bool,bool> NullableVector<bool>::minMaxValues(unsigned int index)
{
    bool set = false;
    char min, max;

    for (; index < data_.size(); ++index)
    {
        if (!isNull(index))  // not for null
        {
            if (!set)
            {
                min = (char)data_.at(index);
                max = (char)data_.at(index);
                set = true;
            }
            else
            {
                min = std::min(min, (char)data_.at(index));
                max = std::max(max, (char)data_.at(index));
            }
        }
    }

    return std::tuple<bool,bool,bool> {set, min, max};
}

template <class T>
std::tuple<bool,T,T> NullableVector<T>::minMaxValuesSorted(unsigned int index)
{
    bool min_set = false;
    bool max_set = false;
    T min, max;

    if (!data_.size())
        return std::tuple<bool,T,T> {min_set && max_set, min, max};

    for (unsigned int tmp_index=index; tmp_index < data_.size(); ++tmp_index)
    {
        if (!isNull(tmp_index))  // not for null
        {
            if (!min_set)
            {
                min = data_.at(tmp_index);
                min_set = true;
                break;
            }
        }
    }

    for (int tmp_index=(int) data_.size()-1; tmp_index >= 0 && tmp_index >= (int)index; --tmp_index)
    {
        //loginf << "tmp_index " << tmp_index << " index " << index << " size " << data_.size();

        if (!isNull(tmp_index))  // not for null
        {
            if (!max_set)
            {
                max = data_.at(tmp_index);
                max_set = true;
                break;
            }
        }
    }

    return std::tuple<bool,T,T> {min_set && max_set, min, max};
}

template <class T>
std::map<boost::optional<T>, std::vector<unsigned int>> NullableVector<T>::distinctValuesWithIndexes(
        unsigned int from_index, unsigned int to_index)
{
    logdbg2 << property_.name();

    std::map<boost::optional<T>, std::vector<unsigned int>> values;

    traced_assert(from_index <= to_index);

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(from_index <= to_index);
        traced_assert(from_index < buffer_.size_);
        traced_assert(to_index < buffer_.size_);
        traced_assert(data_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
    }

    if (from_index + 1 > data_.size())  // no data
        return values;

    for (unsigned int index = from_index; index <= to_index; ++index)
    {
        if (BUFFER_PEDANTIC_CHECKING)
            traced_assert(index < data_.size());

        if (isNull(index))
            values[{}].push_back(index);
        else
            values[data_.at(index)].push_back(index);
    }

    logdbg2 << property_.name() << ": done with "
           << values.size();
    return values;
}

template <class T>
std::map<boost::optional<T>, std::vector<unsigned int>> NullableVector<T>::distinctValuesWithIndexes(
        const std::vector<unsigned int>& indexes)
{
    logdbg2 << property_.name();

    std::map<boost::optional<T>, std::vector<unsigned int>> values;

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(data_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
    }

    for (auto index : indexes)
    {
        if (BUFFER_PEDANTIC_CHECKING)
            traced_assert(index < data_.size());

        if (isNull(index))
            values[{}].push_back(index);
        else
            values[data_.at(index)].push_back(index);
    }

    logdbg2 << property_.name() << ": done with "
           << values.size();
    return values;
}

template <class T>
std::map<T, unsigned int> NullableVector<T>::uniqueValuesWithIndexes()
{
    logdbg2 << property_.name();

    std::map<T, unsigned int> value_indexes;

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(data_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
    }

    for (unsigned int index = 0; index < data_.size(); ++index)
    {
        if (!isNull(index))  // not for null
        {
            if (BUFFER_PEDANTIC_CHECKING)
                traced_assert(index < data_.size());

            traced_assert(!value_indexes.count(data_.at(index)));
            value_indexes[data_.at(index)] = index;
        }
    }

    logdbg2 << property_.name() << ": done with "
           << value_indexes.size();
    return value_indexes;
}

template <class T>
std::map<T, unsigned int> NullableVector<T>::uniqueValuesWithIndexes(const std::set<T>& values)
{
    logdbg2 << property_.name();

    std::map<T, unsigned int> value_indexes;

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(data_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
    }

    for (unsigned int index = 0; index < data_.size(); ++index)
    {
        if (!isNull(index) && values.count(data_.at(index)))  // not for null
        {
            if (BUFFER_PEDANTIC_CHECKING)
                traced_assert(index < data_.size());

            traced_assert(!value_indexes.count(data_.at(index)));
            value_indexes[data_.at(index)] = index;
        }
    }

    logdbg2 << property_.name() << ": done with "
           << value_indexes.size();
    return value_indexes;
}

template <class T>
void NullableVector<T>::convertToStandardFormat(const std::string& from_format)
{
    logdbg2 << property_.name();

    static_assert(std::is_integral<T>::value, "only defined for integer types");

    // std::string value_str;
    // T value;

    if (from_format != "octal")
    {
        logerr << "unknown format '" << from_format
               << "'";
        traced_assert(false);
    }

    unsigned int data_size = data_.size();

    //    tbb::parallel_for(uint(0), data_size, [&](unsigned int cnt) {
    //        if (!isNull(cnt))
    //        {
    //            // value_str = std::to_string(data_.at(cnt));
    //            data_.at(cnt) = std::stoi(std::to_string(data_.at(cnt)), 0, 8);
    //        }
    //    });

    for (unsigned int cnt=0; cnt < data_size; ++cnt)
    {
        if (!isNull(cnt))
            data_.at(cnt) = std::stoi(std::to_string(data_.at(cnt)), 0, 8);
    }

    //    for (unsigned int cnt=0; cnt < data_size; cnt++)
    //    {
    //        if (isNull(cnt))
    //            continue;

    //        value_str = std::to_string(data_.at(cnt));

    //        if (from_format == "octal")
    //        {
    //            data_.at(cnt) = std::stoi(value_str, 0, 8);
    //        }
    //        else
    //        {
    //            logerr << "unknown format '" <<
    //            from_format << "'"; traced_assert(false);
    //        }
    //    }
}

template <class T>
unsigned int NullableVector<T>::contentSize()
{
    return data_.size();
}

template <class T>
void NullableVector<T>::cutToSize(unsigned int size)
{
    logdbg2 << property_.name() << ": size " << size;

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(data_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
    }

    while (null_flags_.size() > size)
        null_flags_.pop_back();

    while (data_.size() > size)
        data_.pop_back();

    // size set in Buffer::cutToSize
}

template <class T>
void NullableVector<T>::cutUpToIndex(unsigned int index) // everything up to index is removed
{
    if (BUFFER_PEDANTIC_CHECKING)
    {
        loginf << "index " << index << " data_size " << data_.size()
               << " null_size " << null_flags_.size();
    }

    // Erase the range including both 0 and index

    if (null_flags_.size())
    {
        if (index < null_flags_.size() - 1)
            null_flags_.erase(null_flags_.begin(), null_flags_.begin() + index + 1); // first and last, is correct
        else
            null_flags_.clear(); // would have been removed
    }

    if (data_.size())
    {
        if (index < data_.size() - 1)
            data_.erase(data_.begin(), data_.begin() + index + 1);
        else
            data_.clear(); // would have been removed
    }

    if (BUFFER_PEDANTIC_CHECKING)
    {
        loginf << "after erase index " << index << " data_size " << data_.size()
               << " null_size " << null_flags_.size();
    }

    // size set in Buffer::cutUpToIndex
}

template <class T>
void NullableVector<T>::removeIndexes(const std::vector<unsigned int>& indexes_to_remove)
{
    // iterator to size cnt

//    {

//        for (auto index_it = indexes_to_remove.rbegin(); index_it != indexes_to_remove.rend(); ++index_it)
//        {
//            if (*index_it < null_flags_copy.size())
//                null_flags_copy.erase(null_flags_copy.begin() + *index_it);

//            if (*index_it < data_copy.size())
//                data_copy.erase(data_copy.begin() + *index_it);
//        }

//    }

    {
        unsigned int data_rm_cnt = 0;
        unsigned int data_idx_old = 0; //old index in data to copy from
        unsigned int data_idx_new = 0; //new index in data to copy to

        //for all indices to be removed... in data
        for (unsigned int i = 0; i < indexes_to_remove.size(); ++i)
        {
            const unsigned int idx_tbr = indexes_to_remove[ i ];


            //..copy from current index in data up to item to be removed into new position

            if (idx_tbr < data_.size())
            {
                while (data_idx_old < idx_tbr)
                {
                    data_[ data_idx_new ] = data_[ data_idx_old ];

                    data_idx_new++;
                    data_idx_old++;
                }

                //skip index to be removed
                ++data_idx_old;
                data_rm_cnt++; // count how many where removed
            }
        }

        //copy any data beyond last index to be removed
        while (data_idx_old < data_.size())
        {
            data_[ data_idx_new ] = data_[ data_idx_old ];

            data_idx_new++;
            data_idx_old++;
        }

        //chop remaining unneeded space
        traced_assert(data_rm_cnt <= data_.size());
        data_.resize(data_.size() - data_rm_cnt);
    }

    {
        unsigned int null_rm_cnt = 0;

        unsigned int null_idx_old = 0; //old index in data to copy from
        unsigned int null_idx_new = 0; //new index in data to copy to

        //for all indices to be removed... in null
        for (unsigned int i = 0; i < indexes_to_remove.size(); ++i)
        {
            const unsigned int idx_tbr = indexes_to_remove[ i ];

            //..copy from current index in null up to item to be removed into new position

            if (idx_tbr < null_flags_.size())
            {
                while (null_idx_old < idx_tbr)
                {
                    null_flags_[ null_idx_new ] = null_flags_[ null_idx_old ];

                    null_idx_new++;
                    null_idx_old++;
                }

                //skip index to be removed
                ++null_idx_old;
                null_rm_cnt++; // count how many where removed
            }
        }

        //copy any null beyond last index to be removed
        while (null_idx_old < null_flags_.size())
        {
            null_flags_[ null_idx_new ] = null_flags_[ null_idx_old ];

            null_idx_new++;
            null_idx_old++;
        }

        traced_assert(null_rm_cnt <= null_flags_.size());
        null_flags_.resize(null_flags_.size() - null_rm_cnt);
    }
}

template <class T>
bool NullableVector<T>::isAlwaysNull() const
{
    logdbg2 << property_.name();

    if (data_.size() == 0)
        return true;


    for (unsigned int cnt = 0; cnt < buffer_.size_; cnt++)
    {
        if (!isNull(cnt))
            return false;
    }

    return true;
}

template <class T>
bool NullableVector<T>::isNeverNull() const
{
    logdbg2 << property_.name();

    for (unsigned int cnt = 0; cnt < buffer_.size_; cnt++)
    {
        if (isNull(cnt))
            return false;
    }

    return true;
}

template <class T>
void NullableVector<T>::swapData (unsigned int index1, unsigned int index2)
{
    bool index1_null = isNull(index1);
    bool index2_null = isNull(index2);

    if (index1_null && index2_null)
        return;
    else if (!index1_null && !index2_null)
    {
        traced_assert(index1 < data_.size());
        traced_assert(index2 < data_.size());

        T val = get(index1);
        set(index1, get(index2));
        set(index2, val);
    }
    else if (index1_null && !index2_null)
    {
        traced_assert(index2 < data_.size());

        set(index1, get(index2));
        setNull(index2);
    }
    else if (!index1_null && index2_null)
    {
        traced_assert(index1 < data_.size());

        set(index2, get(index1));
        setNull(index1);
    }
}

//from https://stackoverflow.com/questions/17074324/how-can-i-sort-two-vectors-in-the-same-way-with-criteria-that-uses-only-one-of

template <class T>
std::vector<unsigned int> NullableVector<T>::sortPermutation()
{
    //assert (isNeverNull());

    //loginf << "UGA data size " << buffer_.size();

    if (data_.size() < buffer_.size())
        resizeDataTo(buffer_.size());

    traced_assert(data_.size() == buffer_.size());
    std::vector<unsigned int> p (data_.size());

    std::iota(p.begin(), p.end(), 0);
    std::sort(p.begin(), p.end(),
              [&](unsigned int i, unsigned int j){

        bool is_i_null = isNull(i);
        bool is_j_null = isNull(j);

        if (is_i_null && is_j_null)
            return false; // same not smaller
        else if (!is_i_null && is_j_null)
            return false; // not null < null = false
        else if (is_i_null && !is_j_null)
            return true; // null < not null = true
        else
            return data_.at(i) < data_.at(j);
    });
    return p;
}

template <class T>
void NullableVector<T>::sortByPermutation(const std::vector<unsigned int>& perm)
{
    //    std::vector<bool> done(data_.size());

    std::vector<bool> done(perm.size());

    //    for (unsigned int i = 0; i < data_.size(); ++i)

    for (unsigned int i = 0; i < perm.size(); ++i)
    {
        if (done.at(i))
            continue;

        traced_assert(i < done.size());
        done.at(i) = true;
        unsigned int prev_j = i;

        traced_assert(i < perm.size());
        unsigned int j = perm.at(i);
        while (i != j)
        {
            //std::swap(data_[prev_j], data_[j]);
            swapData(prev_j, j);

            traced_assert(j < done.size());
            done.at(j) = true;
            prev_j = j;
            traced_assert(j < perm.size());
            j = perm.at(j);
        }
    }


    //    for (unsigned int i = 0; i < data_.size(); ++i)
    //    {
    //        if (done[i])
    //        {
    //            continue;
    //        }
    //        done[i] = true;
    //        unsigned int prev_j = i;
    //        unsigned int j = perm[i];
    //        while (i != j)
    //        {
    //            //std::swap(data_[prev_j], data_[j]);
    //            swapData(prev_j, j);

    //            done[j] = true;
    //            prev_j = j;
    //            j = perm[j];
    //        }
    //    }
}

template <class T>
nlohmann::json NullableVector<T>::asJSON(unsigned int max_size)
{
    nlohmann::json list = nlohmann::json::array();

    unsigned int size = buffer_.size();

    if (max_size != 0)
        size = std::min(size, max_size);

    for (unsigned int cnt=0; cnt < size; ++cnt)
    {
        if (isNull(cnt))
            list.push_back(nlohmann::json());
        else
            list.push_back(get(cnt));
    }

    return list;
}

// private stuff

/// @brief Sets specific element to not Null value
template <class T>
void NullableVector<T>::unsetNull(unsigned int index)
{
    logdbg2 << property_.name();

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(data_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
        traced_assert(index < buffer_.size_);
        traced_assert(index < data_.size());
    }

    if (index < null_flags_.size())  // if was already set
        null_flags_.at(index) = false;
}

template <>
NullableVector<bool>& NullableVector<bool>::operator*=(double factor);

template <>
void NullableVector<bool>::append(unsigned int index, bool value);

template <>
void NullableVector<std::string>::append(unsigned int index, std::string value);

template <>
nlohmann::json NullableVector<boost::posix_time::ptime>::asJSON(unsigned int max_size);


