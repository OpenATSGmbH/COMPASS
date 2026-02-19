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

#include <algorithm>
#include <cstdint>
#include <numeric>
#include <unordered_map>

/**
 * Full specialization of NullableVector for std::string using dictionary encoding.
 *
 * Instead of storing one std::string per row, keeps a small table of unique strings
 * (dictionary_) and per-row uint32 indices into that table. Optimized for columns
 * with high repetition (K unique values << N rows).
 *
 * Interface matches the generic NullableVector<T> for drop-in use via Buffer.
 */
template <>
class NullableVector<std::string>
{
    friend class Buffer;

public:
    virtual ~NullableVector() {}

    void renameProperty(const std::string& name) { property_.rename(name); }

    void clear();
    void clearData();

    const std::string get(unsigned int index) const;
    const std::string& getRef(unsigned int index) const;
    const std::string getAsString(unsigned int index) const;

    void set(unsigned int index, std::string value);
    void setFromFormat(unsigned int index, const std::string& format,
                       const std::string& value_str, bool debug = false);
    void setAll(std::string value);

    void bulkSet(unsigned int dst_offset, const void* src, const uint64_t* validity,
                 size_t src_offset, size_t count);

    void ensureMinSize(unsigned int size);

    void append(unsigned int index, std::string value);
    void appendFromFormat(unsigned int index, const std::string& format,
                          const std::string& value_str);

    void setNull(unsigned int index);
    void setAllNull();

    NullableVector<std::string>& operator*=(double factor);

    std::set<std::string> distinctValues(unsigned int index = 0);
    std::map<std::string, unsigned int> distinctValuesWithCounts(unsigned int index = 0);
    std::tuple<bool, std::string, std::string> minMaxValues(unsigned int index = 0);
    std::tuple<bool, std::string, std::string> minMaxValuesSorted(unsigned int index = 0);

    std::map<boost::optional<std::string>, std::vector<unsigned int>> distinctValuesWithIndexes(
        unsigned int from_index, unsigned int to_index);
    std::map<boost::optional<std::string>, std::vector<unsigned int>> distinctValuesWithIndexes(
        const std::vector<unsigned int>& indexes);

    std::map<std::string, unsigned int> uniqueValuesWithIndexes();
    std::map<std::string, unsigned int> uniqueValuesWithIndexes(const std::set<std::string>& values);

    void convertToStandardFormat(const std::string& from_format);

    unsigned int contentSize();

    bool isNull(unsigned int index) const;
    bool isAlwaysNull() const;
    bool isNeverNull() const;

    void swapData(unsigned int index1, unsigned int index2);

    std::string propertyName() const { return property_.name(); }
    std::string propertyID() const { return property_.name() + "(" + property_.dataTypeString() + ")"; }

    std::vector<unsigned int> sortPermutation();
    void sortByPermutation(const std::vector<unsigned int>& perm);

    nlohmann::json asJSON(unsigned int max_size = 0);

    // dictionary-specific accessor
    size_t dictionarySize() const { return dictionary_.size(); }

private:
    Property property_;
    Buffer& buffer_;

    std::vector<std::string>                  dictionary_;
    std::unordered_map<std::string, uint32_t> lookup_;
    std::vector<uint32_t>                     indices_;
    std::vector<bool>                         null_flags_;

    uint32_t getOrInsert(const std::string& value);

    void unsetNull(unsigned int index);
    void resizeIndicesTo(unsigned int size);
    void resizeNullTo(unsigned int size);
    void addData(NullableVector<std::string>& other);
    void copyData(NullableVector<std::string>& other);
    void cutToSize(unsigned int size);
    void cutUpToIndex(unsigned int index);
    void removeIndexes(const std::vector<unsigned int>& indexes_to_remove);

    NullableVector(Property& property, Buffer& buffer);
};

// ============================================================================
// Inline implementations
// ============================================================================

inline NullableVector<std::string>::NullableVector(Property& property, Buffer& buffer)
    : property_(property), buffer_(buffer)
{
}

inline void NullableVector<std::string>::clear()
{
    std::fill(indices_.begin(), indices_.end(), 0);
    std::fill(null_flags_.begin(), null_flags_.end(), true);
}

inline void NullableVector<std::string>::clearData()
{
    dictionary_.clear();
    lookup_.clear();
    indices_.clear();
    null_flags_.clear();
}

inline const std::string NullableVector<std::string>::get(unsigned int index) const
{
    logdbg2 << property_.name() << ": index " << index;
    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(indices_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
        traced_assert(index < indices_.size());
    }

    if (isNull(index))
    {
        logerr << property_.name() << ": get: index " << index << " is null";
        traced_assert(false);
    }

    return dictionary_[indices_[index]];
}

inline const std::string& NullableVector<std::string>::getRef(unsigned int index) const
{
    logdbg2 << property_.name() << ": index " << index;
    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(indices_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
        traced_assert(index < indices_.size());
    }

    if (isNull(index))
    {
        logerr << property_.name() << ": getRef: index " << index << " is null";
        traced_assert(false);
    }

    return dictionary_[indices_[index]];
}

inline const std::string NullableVector<std::string>::getAsString(unsigned int index) const
{
    return get(index);
}

inline void NullableVector<std::string>::set(unsigned int index, std::string value)
{
    logdbg2 << property_.name() << ": index " << index << " value '" << value << "'";

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(indices_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
    }

    if (index >= indices_.size())
    {
        if (index != indices_.size())
            resizeNullTo(index + 1);

        resizeIndicesTo(index + 1);
    }

    indices_[index] = getOrInsert(value);
    unsetNull(index);
}

inline void NullableVector<std::string>::setFromFormat(unsigned int index, const std::string& format,
                                                       const std::string& value_str, bool debug)
{
    logerr << "setFromFormat not applicable for string type, format '" << format << "'";
    traced_assert(false);
}

inline void NullableVector<std::string>::setAll(std::string value)
{
    uint32_t idx = getOrInsert(value);
    unsigned int data_size = indices_.size();

    for (unsigned int i = 0; i < data_size; ++i)
    {
        indices_[i] = idx;
        unsetNull(i);
    }
}

inline void NullableVector<std::string>::bulkSet(unsigned int dst_offset, const void* src,
                                                  const uint64_t* validity,
                                                  size_t src_offset, size_t count)
{
    // bulk memcpy not applicable for dictionary-encoded strings;
    // use set() and setNull() individually instead
    logerr << "bulkSet not supported for dictionary-encoded string NullableVector";
    traced_assert(false);
}

inline void NullableVector<std::string>::ensureMinSize(unsigned int size)
{
    if (size > indices_.size())
        resizeIndicesTo(size);
}

inline void NullableVector<std::string>::append(unsigned int index, std::string value)
{
    logdbg2 << property_.name() << ": index " << index << " value '" << value << "'";

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(indices_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
    }

    if (index >= indices_.size())
    {
        if (index != indices_.size())
            resizeNullTo(index + 1);

        resizeIndicesTo(index + 1);

        // newly created entry — just set the value
        indices_[index] = getOrInsert(value);
        unsetNull(index);
        return;
    }

    if (isNull(index))
    {
        indices_[index] = getOrInsert(value);
    }
    else
    {
        const std::string& current = dictionary_[indices_[index]];
        std::string combined;
        if (current.size())
            combined = current + ";" + value;
        else
            combined = value;
        indices_[index] = getOrInsert(combined);
    }
    unsetNull(index);
}

inline void NullableVector<std::string>::appendFromFormat(unsigned int index,
                                                          const std::string& format,
                                                          const std::string& value_str)
{
    logerr << "appendFromFormat not applicable for string type, format '" << format << "'";
    traced_assert(false);
}

inline void NullableVector<std::string>::setNull(unsigned int index)
{
    logdbg2 << property_.name() << ": index " << index;

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(indices_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
    }

    if (index >= null_flags_.size())
        resizeNullTo(index + 1);

    null_flags_[index] = true;
}

inline void NullableVector<std::string>::setAllNull()
{
    unsigned int data_size = indices_.size();

    for (unsigned int cnt = 0; cnt < data_size; ++cnt)
        setNull(cnt);
}

inline NullableVector<std::string>& NullableVector<std::string>::operator*=(double factor)
{
    logerr << "operator*= not applicable for string NullableVector";
    traced_assert(false);
    return *this;
}

inline std::set<std::string> NullableVector<std::string>::distinctValues(unsigned int index)
{
    logdbg2 << property_.name();

    std::vector<bool> used(dictionary_.size(), false);

    for (unsigned int i = index; i < indices_.size(); ++i)
    {
        if (!isNull(i))
            used[indices_[i]] = true;
    }

    std::set<std::string> values;
    for (uint32_t j = 0; j < dictionary_.size(); ++j)
    {
        if (used[j])
            values.insert(dictionary_[j]);
    }
    return values;
}

inline std::map<std::string, unsigned int> NullableVector<std::string>::distinctValuesWithCounts(
    unsigned int index)
{
    logdbg2 << property_.name();

    std::vector<unsigned int> counts(dictionary_.size(), 0);

    for (unsigned int i = index; i < indices_.size(); ++i)
    {
        if (!isNull(i))
            counts[indices_[i]]++;
    }

    std::map<std::string, unsigned int> values;
    for (uint32_t j = 0; j < dictionary_.size(); ++j)
    {
        if (counts[j] > 0)
            values[dictionary_[j]] = counts[j];
    }
    return values;
}

inline std::tuple<bool, std::string, std::string> NullableVector<std::string>::minMaxValues(
    unsigned int index)
{
    bool found = false;
    std::string min_val, max_val;

    std::vector<bool> used(dictionary_.size(), false);
    for (unsigned int i = index; i < indices_.size(); ++i)
    {
        if (!isNull(i))
            used[indices_[i]] = true;
    }

    for (uint32_t j = 0; j < dictionary_.size(); ++j)
    {
        if (!used[j])
            continue;

        if (!found)
        {
            min_val = dictionary_[j];
            max_val = dictionary_[j];
            found = true;
        }
        else
        {
            if (dictionary_[j] < min_val) min_val = dictionary_[j];
            if (dictionary_[j] > max_val) max_val = dictionary_[j];
        }
    }

    return std::make_tuple(found, min_val, max_val);
}

inline std::tuple<bool, std::string, std::string> NullableVector<std::string>::minMaxValuesSorted(
    unsigned int index)
{
    return minMaxValues(index);
}

inline std::map<boost::optional<std::string>, std::vector<unsigned int>>
NullableVector<std::string>::distinctValuesWithIndexes(unsigned int from_index,
                                                       unsigned int to_index)
{
    logdbg2 << property_.name();

    traced_assert(from_index <= to_index);

    std::map<boost::optional<std::string>, std::vector<unsigned int>> values;

    if (from_index >= indices_.size())
        return values;

    for (unsigned int i = from_index; i <= to_index; ++i)
    {
        if (isNull(i))
            values[{}].push_back(i);
        else
            values[dictionary_[indices_[i]]].push_back(i);
    }

    logdbg2 << property_.name() << ": done with " << values.size();
    return values;
}

inline std::map<boost::optional<std::string>, std::vector<unsigned int>>
NullableVector<std::string>::distinctValuesWithIndexes(const std::vector<unsigned int>& indexes)
{
    logdbg2 << property_.name();

    std::map<boost::optional<std::string>, std::vector<unsigned int>> values;

    for (auto index : indexes)
    {
        if (isNull(index))
            values[{}].push_back(index);
        else
            values[dictionary_[indices_[index]]].push_back(index);
    }

    logdbg2 << property_.name() << ": done with " << values.size();
    return values;
}

inline std::map<std::string, unsigned int> NullableVector<std::string>::uniqueValuesWithIndexes()
{
    logdbg2 << property_.name();

    std::map<std::string, unsigned int> value_indexes;

    for (unsigned int i = 0; i < indices_.size(); ++i)
    {
        if (!isNull(i))
        {
            const auto& val = dictionary_[indices_[i]];
            traced_assert(!value_indexes.count(val));
            value_indexes[val] = i;
        }
    }

    logdbg2 << property_.name() << ": done with " << value_indexes.size();
    return value_indexes;
}

inline std::map<std::string, unsigned int> NullableVector<std::string>::uniqueValuesWithIndexes(
    const std::set<std::string>& values)
{
    logdbg2 << property_.name();

    std::map<std::string, unsigned int> value_indexes;

    for (unsigned int i = 0; i < indices_.size(); ++i)
    {
        if (!isNull(i))
        {
            const auto& val = dictionary_[indices_[i]];
            if (values.count(val))
            {
                traced_assert(!value_indexes.count(val));
                value_indexes[val] = i;
            }
        }
    }

    logdbg2 << property_.name() << ": done with " << value_indexes.size();
    return value_indexes;
}

inline void NullableVector<std::string>::convertToStandardFormat(const std::string& from_format)
{
    logerr << "convertToStandardFormat not applicable for string type, format '"
           << from_format << "'";
    traced_assert(false);
}

inline unsigned int NullableVector<std::string>::contentSize()
{
    return indices_.size();
}

inline bool NullableVector<std::string>::isNull(unsigned int index) const
{
    logdbg2 << property_.name() << ": index " << index;

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(indices_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
        traced_assert(index < buffer_.size_);
    }

    if (index < null_flags_.size())
        return null_flags_[index];

    // null not stored, so all set are not null
    if (index >= indices_.size())
        return true;

    // must be set
    return false;
}

inline bool NullableVector<std::string>::isAlwaysNull() const
{
    logdbg2 << property_.name();

    if (indices_.size() == 0)
        return true;

    for (unsigned int cnt = 0; cnt < buffer_.size_; cnt++)
    {
        if (!isNull(cnt))
            return false;
    }

    return true;
}

inline bool NullableVector<std::string>::isNeverNull() const
{
    logdbg2 << property_.name();

    for (unsigned int cnt = 0; cnt < buffer_.size_; cnt++)
    {
        if (isNull(cnt))
            return false;
    }

    return true;
}

inline void NullableVector<std::string>::swapData(unsigned int index1, unsigned int index2)
{
    bool null1 = isNull(index1);
    bool null2 = isNull(index2);

    if (null1 && null2)
        return;
    else if (!null1 && !null2)
    {
        traced_assert(index1 < indices_.size());
        traced_assert(index2 < indices_.size());
        std::swap(indices_[index1], indices_[index2]);
    }
    else if (null1 && !null2)
    {
        traced_assert(index2 < indices_.size());
        if (index1 >= indices_.size())
            resizeIndicesTo(index1 + 1);
        indices_[index1] = indices_[index2];
        unsetNull(index1);
        setNull(index2);
    }
    else // !null1 && null2
    {
        traced_assert(index1 < indices_.size());
        if (index2 >= indices_.size())
            resizeIndicesTo(index2 + 1);
        indices_[index2] = indices_[index1];
        unsetNull(index2);
        setNull(index1);
    }
}

inline std::vector<unsigned int> NullableVector<std::string>::sortPermutation()
{
    if (indices_.size() < buffer_.size_)
        resizeIndicesTo(buffer_.size_);

    traced_assert(indices_.size() == buffer_.size_);
    std::vector<unsigned int> p(indices_.size());
    std::iota(p.begin(), p.end(), 0);

    std::sort(p.begin(), p.end(),
              [this](unsigned int i, unsigned int j)
    {
        bool ni = isNull(i);
        bool nj = isNull(j);

        if (ni && nj) return false;
        if (ni) return true;   // null < non-null
        if (nj) return false;

        return dictionary_[indices_[i]] < dictionary_[indices_[j]];
    });
    return p;
}

inline void NullableVector<std::string>::sortByPermutation(const std::vector<unsigned int>& perm)
{
    std::vector<bool> done(perm.size());

    for (unsigned int i = 0; i < perm.size(); ++i)
    {
        if (done[i])
            continue;

        done[i] = true;
        unsigned int prev_j = i;
        unsigned int j = perm[i];

        while (i != j)
        {
            swapData(prev_j, j);
            done[j] = true;
            prev_j = j;
            j = perm[j];
        }
    }
}

inline nlohmann::json NullableVector<std::string>::asJSON(unsigned int max_size)
{
    nlohmann::json list = nlohmann::json::array();

    unsigned int size = buffer_.size();

    if (max_size != 0)
        size = std::min(size, max_size);

    for (unsigned int cnt = 0; cnt < size; ++cnt)
    {
        if (isNull(cnt))
            list.push_back(nlohmann::json());
        else
            list.push_back(get(cnt));
    }

    return list;
}

// ============================================================================
// Private methods
// ============================================================================

inline uint32_t NullableVector<std::string>::getOrInsert(const std::string& value)
{
    auto it = lookup_.find(value);
    if (it != lookup_.end())
        return it->second;

    uint32_t idx = static_cast<uint32_t>(dictionary_.size());
    dictionary_.push_back(value);
    lookup_[value] = idx;
    return idx;
}

inline void NullableVector<std::string>::unsetNull(unsigned int index)
{
    logdbg2 << property_.name();

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(indices_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
        traced_assert(index < buffer_.size_);
        traced_assert(index < indices_.size());
    }

    if (index < null_flags_.size())
        null_flags_[index] = false;
}

inline void NullableVector<std::string>::resizeIndicesTo(unsigned int size)
{
    logdbg2 << property_.name() << ": size " << size;

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(indices_.size() <= buffer_.size_);
        traced_assert(indices_.size() < size);
    }

    indices_.resize(size, 0);

    if (buffer_.size_ < indices_.size())
        buffer_.size_ = indices_.size();
}

inline void NullableVector<std::string>::resizeNullTo(unsigned int size)
{
    logdbg2 << property_.name() << ": size " << size;

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(size >= null_flags_.size());
        traced_assert(null_flags_.size() <= buffer_.size_);
    }

    if (indices_.size() > null_flags_.size())
        null_flags_.resize(indices_.size(), false);

    if (null_flags_.size() < size)
        null_flags_.resize(size, true);

    if (buffer_.size_ < null_flags_.size())
        buffer_.size_ = null_flags_.size();

    if (BUFFER_PEDANTIC_CHECKING)
        traced_assert(null_flags_.size() >= size);
}

inline void NullableVector<std::string>::addData(NullableVector<std::string>& other)
{
    logdbg2 << property_.name();

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(indices_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
    }

    // empty dictionary means all entries are null — indices may contain junk 0s
    // from ensureMinSize/resizeIndicesTo, so skip index remapping entirely
    if (other.dictionary_.empty())
    {
        logdbg2 << property_.name() << ": other dict empty, treating as all-null";
        resizeNullTo(buffer_.size_);

        if (other.null_flags_.size())
        {
            null_flags_.insert(null_flags_.end(), other.null_flags_.begin(),
                               other.null_flags_.end());
        }
        else
        {
            // no null flags and no dictionary — synthesise nulls for other's rows
            null_flags_.resize(null_flags_.size() + other.buffer_.size_, true);
        }
        return;
    }

    if (!other.indices_.size() && other.null_flags_.size())
    {
        logdbg2 << property_.name() << ": 1: other no data resizing null";
        resizeNullTo(buffer_.size_);
        logdbg2 << property_.name() << ": 1: inserting null";
        null_flags_.insert(null_flags_.end(), other.null_flags_.begin(), other.null_flags_.end());
        return;
    }

    // build index remapping: other dict index -> our dict index
    std::vector<uint32_t> remap(other.dictionary_.size());
    for (uint32_t j = 0; j < other.dictionary_.size(); ++j)
        remap[j] = getOrInsert(other.dictionary_[j]);

    if (other.indices_.size() && !other.null_flags_.size())
    {
        logdbg2 << property_.name() << ": 2: other has everything set";

        if (indices_.size() < buffer_.size_)
        {
            logdbg2 << property_.name() << ": 2: data not full, setting null";
            resizeNullTo(buffer_.size_);

            logdbg2 << property_.name() << ": 2: resizing data";
            resizeIndicesTo(buffer_.size_);
        }

        logdbg2 << property_.name() << ": 2: inserting data";
        indices_.reserve(indices_.size() + other.indices_.size());
        for (auto idx : other.indices_)
            indices_.push_back(remap[idx]);
        return;
    }

    logdbg2 << property_.name() << ": 3: mixture, both have data & nulls";

    logdbg2 << property_.name() << ": 3: resizing null to " << buffer_.size_;
    resizeNullTo(buffer_.size_);
    logdbg2 << property_.name() << ": 3: inserting nulls";
    null_flags_.insert(null_flags_.end(), other.null_flags_.begin(), other.null_flags_.end());

    if (indices_.size() < buffer_.size_)
    {
        logdbg2 << property_.name() << ": 3: resizing data";
        resizeIndicesTo(buffer_.size_);
    }

    logdbg2 << property_.name() << ": 3: inserting data";
    indices_.reserve(indices_.size() + other.indices_.size());
    for (auto idx : other.indices_)
        indices_.push_back(remap[idx]);

    // size adjusted in Buffer::seizeBuffer
    logdbg2 << property_.name() << ": end";
}

inline void NullableVector<std::string>::copyData(NullableVector<std::string>& other)
{
    logdbg2 << property_.name();

    dictionary_ = other.dictionary_;
    lookup_ = other.lookup_;
    indices_ = other.indices_;
    null_flags_ = other.null_flags_;

    if (buffer_.size_ < indices_.size())
        buffer_.size_ = indices_.size();

    if (buffer_.size_ < null_flags_.size())
        buffer_.size_ = null_flags_.size();

    logdbg2 << property_.name() << ": end";
}

inline void NullableVector<std::string>::cutToSize(unsigned int size)
{
    logdbg2 << property_.name() << ": size " << size;

    if (BUFFER_PEDANTIC_CHECKING)
    {
        traced_assert(indices_.size() <= buffer_.size_);
        traced_assert(null_flags_.size() <= buffer_.size_);
    }

    while (null_flags_.size() > size)
        null_flags_.pop_back();

    while (indices_.size() > size)
        indices_.pop_back();

    // size set in Buffer::cutToSize
}

inline void NullableVector<std::string>::cutUpToIndex(unsigned int index)
{
    if (BUFFER_PEDANTIC_CHECKING)
    {
        loginf << "index " << index << " indices_size " << indices_.size()
               << " null_size " << null_flags_.size();
    }

    if (null_flags_.size())
    {
        if (index < null_flags_.size() - 1)
            null_flags_.erase(null_flags_.begin(), null_flags_.begin() + index + 1);
        else
            null_flags_.clear();
    }

    if (indices_.size())
    {
        if (index < indices_.size() - 1)
            indices_.erase(indices_.begin(), indices_.begin() + index + 1);
        else
            indices_.clear();
    }

    if (BUFFER_PEDANTIC_CHECKING)
    {
        loginf << "after erase index " << index << " indices_size " << indices_.size()
               << " null_size " << null_flags_.size();
    }

    // size set in Buffer::cutUpToIndex
}

inline void NullableVector<std::string>::removeIndexes(
    const std::vector<unsigned int>& indexes_to_remove)
{
    // compact indices_
    {
        unsigned int rm_cnt = 0;
        unsigned int idx_old = 0;
        unsigned int idx_new = 0;

        for (unsigned int i = 0; i < indexes_to_remove.size(); ++i)
        {
            const unsigned int idx_tbr = indexes_to_remove[i];

            if (idx_tbr < indices_.size())
            {
                while (idx_old < idx_tbr)
                {
                    indices_[idx_new] = indices_[idx_old];
                    idx_new++;
                    idx_old++;
                }
                ++idx_old;
                rm_cnt++;
            }
        }

        while (idx_old < indices_.size())
        {
            indices_[idx_new] = indices_[idx_old];
            idx_new++;
            idx_old++;
        }

        traced_assert(rm_cnt <= indices_.size());
        indices_.resize(indices_.size() - rm_cnt);
    }

    // compact null_flags_
    {
        unsigned int rm_cnt = 0;
        unsigned int idx_old = 0;
        unsigned int idx_new = 0;

        for (unsigned int i = 0; i < indexes_to_remove.size(); ++i)
        {
            const unsigned int idx_tbr = indexes_to_remove[i];

            if (idx_tbr < null_flags_.size())
            {
                while (idx_old < idx_tbr)
                {
                    null_flags_[idx_new] = null_flags_[idx_old];
                    idx_new++;
                    idx_old++;
                }
                ++idx_old;
                rm_cnt++;
            }
        }

        while (idx_old < null_flags_.size())
        {
            null_flags_[idx_new] = null_flags_[idx_old];
            idx_new++;
            idx_old++;
        }

        traced_assert(rm_cnt <= null_flags_.size());
        null_flags_.resize(null_flags_.size() - rm_cnt);
    }
}
