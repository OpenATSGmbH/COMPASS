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

#include "util/timeconv.h"
#include "dbcontent/variable/variable.h"
#include "histogram_raw.h"

#include <string>
#include <vector>
#include <memory>
#include <cmath>

#include <boost/date_time.hpp>
#include <boost/optional.hpp>

namespace histogram_helpers
{
    /**
     * Check for finite values on all needed data types.
     */
    template<typename T>
    inline bool checkFinite(const T& v)
    {
        return std::isfinite(v);
    }
    template<>
    inline bool checkFinite<std::string>(const std::string& v)
    {
        return true;
    }
    template<>
    inline bool checkFinite<boost::posix_time::ptime>(const boost::posix_time::ptime& v)
    {
        return true;
    }
}

/**
 * Templated histogram bin.
 */
template <typename T>
struct HistogramBinT
{
    /**
     * Checks if the given value is inside the bin range.
     */
    bool isInside(const T& v) const
    {
        //default for numerical values
        return isInRange<T>(min_value, max_value, v, max_included);
    }

    void reset() 
    {
        count = 0;
    }

    void add(uint32_t c = 1)
    {
        count += c;
    }

    /**
     */
    std::string label(dbContent::Variable* data_var) const
    {
        return generateLabel(midValue(), data_var);
    }

    /**
     * Generates a label string for the bins minimum value.
     */
    std::string labelMin(dbContent::Variable* data_var) const
    {
        return generateLabel(min_value, data_var);
    }

    /**
     * Generates a label string for the bins maximum value.
     */
    std::string labelMax(dbContent::Variable* data_var) const
    {
        return generateLabel(max_value, data_var);
    }

    /**
     * Returns a representative value for the bin range (in most cases the mid value).
     */
    T midValue() const
    {
        const T mid = (min_value + max_value) / (T)2;
        return mid;
    }

    uint32_t count          = 0;     //bin count
    bool     max_included   = false; //if true bin range is [min,max], [min,max) otherwise
    T        min_value;              //bin min range value
    T        max_value;              //bin max range value

private:
    /**
     * Generates a label string for the bin.
     */
    std::string generateLabel(const T& value, dbContent::Variable* data_var) const
    {
        return std::to_string(value);
    }

    /**
     * Checks if v is inside r_min and r_max.
     */
    template<typename Tr>
    bool isInRange(const Tr& range_min, const Tr& range_max, const Tr& v, bool max_included = true) const
    {
        if(max_included)
            return (v >= range_min && v <= range_max);

        return (v >= range_min && v < range_max);
    }
};

template<>
inline std::string HistogramBinT<std::string>::midValue() const
{
    return min_value;
}
template<>
inline bool HistogramBinT<bool>::midValue() const
{
    return min_value;
}
template<>
inline boost::posix_time::ptime HistogramBinT<boost::posix_time::ptime>::midValue() const
{
    auto tmin = Utils::Time::toLong(min_value);
    auto tmax = Utils::Time::toLong(max_value);
    auto mid  = (tmin + tmax) / (long)2;

    return Utils::Time::fromLong(mid);
}

template<>
inline bool HistogramBinT<std::string>::isInside(const std::string& v) const
{
    return (min_value == v);
}
template<>
inline bool HistogramBinT<boost::posix_time::ptime>::isInside(const boost::posix_time::ptime& v) const
{
    auto tmin = Utils::Time::toLong(min_value);
    auto tmax = Utils::Time::toLong(max_value);
    auto t    = Utils::Time::toLong(v);

    return isInRange<long>(tmin, tmax, t, max_included);
}

template<>
inline std::string HistogramBinT<float>::generateLabel(const float& value, dbContent::Variable* data_var) const
{
    std::string s;
    if (data_var && data_var->representation() != dbContent::Variable::Representation::STANDARD)
        s = data_var->getAsSpecialRepresentationString(value);
    else
        s = std::to_string(value);

    return s;
}
template<>
inline std::string HistogramBinT<double>::generateLabel(const double& value, dbContent::Variable* data_var) const
{
    std::string s;
    if (data_var && data_var->representation() != dbContent::Variable::Representation::STANDARD)
        s = data_var->getAsSpecialRepresentationString(value);
    else
        s = std::to_string(value);

    return s;
}
template<>
inline std::string HistogramBinT<std::string>::generateLabel(const std::string& value, dbContent::Variable* data_var) const
{
    return value;
}
template<>
inline std::string HistogramBinT<bool>::generateLabel(const bool& value, dbContent::Variable* data_var) const
{
    return (value ? "true" : "false");
}
template<>
inline std::string HistogramBinT<boost::posix_time::ptime>::generateLabel(const boost::posix_time::ptime& value, dbContent::Variable* data_var) const
{
    return Utils::Time::toString(value);
}

/**
 * Current histogram configuration.
 */
struct HistogramConfig
{
    static const unsigned int DefaultBins = 20;

    /**
     * Histogram generation type.
     */
    enum class Type
    {
        Range = 0, //the histogram is generated from a certain data range and a number of equally sized bins inbetween
        Category   //the histogram is generated from a fixed number of values that need to match (categories)
    };

    Type          type        = Type::Range; //histogram generation type
    unsigned int  num_bins    = DefaultBins; //number of histogram bins to be generated
    bool          sorted_bins = true;
};

/**
 * A templated histogram.
 */
template<typename T> 
class HistogramT
{
public:
    HistogramT() = default;
    virtual ~HistogramT() = default;

    /**
     * Resets the histogram.
     */
    void clear()
    {
        bins_.clear();
        ranges_.clear();
        categories_.clear();
        step_size_ = {};
        not_found_bin_.reset();

        config_ = {};
        config_.num_bins = 0;
    }

    /**
     * Resets all bins to zero.
     */
    void resetBins()
    {
        for (auto& b : bins_)
            b.reset();
    }

    /**
     */
    size_t numBins() const
    {
        return config_.num_bins;
    }

    /**
     * Returns the current configuration of the histogram.
     */
    HistogramConfig configuration() const
    {
        return config_;
    }

    /**
     * Returns the given bin.
     */
    const HistogramBinT<T>& getBin(size_t idx) const
    {
        traced_assert(idx < bins_.size());
        return bins_.at(idx);
    }

    /**
     * Collects all bin counts.
     */
    std::vector<uint32_t> counts() const
    {
        std::vector<uint32_t> cnts;
        cnts.reserve(bins_.size());

        for (const auto& b : bins_)
            cnts.push_back(b.count);

        return cnts;
    }

    /**
     * Convert to raw histogram.
     */
    RawHistogram toRaw(bool add_invalid_bin = true, dbContent::Variable* data_var = nullptr) const
    {
        RawHistogram::RawHistogramBins bins;
        bins.reserve(bins_.size() + 1);

        auto addBin = [ & ] (const HistogramBinT<T>& b, RawHistogramBin::Tag tag, const std::string& label)
        {
            bins.push_back(RawHistogramBin(b.count, 
                                           label.empty() ? b.label(data_var) : label,
                                           tag,
                                           label.empty() ? b.labelMin(data_var) : label,
                                           label.empty() ? b.labelMax(data_var) : label));
        };

        for (const auto& b : bins_)
            addBin(b, RawHistogramBin::Tag::Standard, "");

        if (add_invalid_bin)
            addBin(not_found_bin_, RawHistogramBin::Tag::CouldNotInsert, "Invalid");

        RawHistogram h;
        h.addBins(bins);

        return h;
    }

    /**
     * Returns the number of values which could not be added to the histogram range.
     */
    uint32_t unassignedCount() const
    {
        return not_found_bin_.count;
    }

    /**
     * Creation from data range.
     * E.g. useful for all numerical ranges.
     */
    bool createFromRange(size_t n, const T& min_value, const T& max_value)
    {
        return createFromRangeT<T>(n, min_value, max_value);
    }

    /**
     * Creation from discrete categories. Values MUST match one of these category values to be added.
     * E.g. useful for strings or small range enums.
     */
    bool createFromCategories(const std::vector<T>& categories, bool categories_are_sorted = false)
    {
        size_t n = categories.size();
        
        clear();

        config_.type        = HistogramConfig::Type::Category;
        config_.num_bins    = n;
        config_.sorted_bins = categories_are_sorted;

        if (n < 1)
            return true;

        bins_.resize(n);

        for (size_t i = 0; i < n; ++i)
        {
            bins_[ i ].min_value    = categories[ i ];
            bins_[ i ].max_value    = categories[ i ];
            bins_[ i ].max_included = true; //in this mode each bin has unique min-max values (and min = max)

            //add to search structure
            categories_[ categories[ i ] ] = (int)i;
        }

        return true;
    }

    /**
     * Add the given value to the histogram.
     */
    int add(const T& v, uint32_t count = 1)
    {
        int bin_idx = findBin(v);

        if (bin_idx < 0)
            logerr << "no bin for value '" << v << "'";

        increment(bin_idx, count);

        return bin_idx;
    }

    /**
     * Add the given values to the histogram.
     */
    void add(const std::vector<T>& values)
    {
        for (const auto& v : values)
            add(v);
    }

    /**
     * Increment the given bin.
     */
    void increment(int idx, uint32_t count = 1) 
    {
        //invalid bin?
        if (idx < 0)
        {
            not_found_bin_.add(count);
            return;
        }

        //increment bin
        bins_.at(idx).add(count);
    }

    /**
     * Tries to find the bin the given value is part of, return -1 if not found.
     */
    int findBin(const T& v) const
    {
        if (!histogram_helpers::checkFinite(v))
            return -2;

        //estimate bin range to search
        auto bin_range = estimateTargetBin(v);

        //no range found?
        if (bin_range.first < 0)
            return -1;

        //search bin range for correct bin
        return findBinLinear(v, &bin_range.first, &bin_range.second);
    }

    /**
     * Zooms to the given bin range, permanently dropping all other (outer) bins.
     */
    bool zoom(unsigned int bin0, unsigned int bin1, const boost::optional<size_t>& n = boost::optional<size_t>())
    {
        if (numBins() < 1 ||
            bin1 < bin0 ||
            bin1 >= numBins())
            return false;

        //all bins selected?
        if (bin0 == 0 && bin1 == numBins() - 1)
            return true;

        if (config_.type == HistogramConfig::Type::Category)
        {
            //recreate histograms from subcategories 
            std::vector<T> categories;
            for (unsigned int i = bin0; i <= bin1; ++i)
                categories.push_back(getBin(i).min_value);

            createFromCategories(categories);
        }
        else // Type::Range
        {
            //recreate histogram from subrange
            const auto& b0 = getBin(bin0);
            const auto& b1 = getBin(bin1);

            auto min_value = b0.min_value;
            auto max_value = b1.max_value;

            createFromRange(n.has_value() ? n.value() : HistogramConfig::DefaultBins, min_value, max_value);
        }

        return true;
    }

    /**
     */
    void print() const
    {
        int cnt = 0;
        for (const auto& b : bins_)
            std::cout << "bin" << cnt++ << " [" << b.min_value << "," << b.max_value << (b.max_included ? ")" : "]") << " #" << b.count << std::endl; 
    }

private:
    /**
     * Convert data to an internal unit.
     * The internal unit should support basic computations such as e.g.
     * comparison to 0, -, / etc.
     */
    template<typename Tinternal>
    Tinternal convertToInternal(const T& v) const
    {
        //by default just try cast
        return (Tinternal)v;
    }

    /**
     * Convert data from an internal unit.
     * The internal unit should support basic computations such as e.g.
     * comparison to 0, -, / etc.
     */
    template<typename Tinternal>
    T convertFromInternal(const Tinternal& v) const
    {
        //by default just try cast
        return (T)v;
    }

    /**
     * Creation of bins from ranges using an internal data type Tinternal for computation
     * of step size, intervals, etc.
     */
    template<typename Tinternal = T>
    bool createFromRangeT(size_t n, const T& min_value, const T& max_value)
    {
        clear();

        if (!histogram_helpers::checkFinite<T>(min_value) || 
            !histogram_helpers::checkFinite<T>(max_value) || 
            min_value >= max_value)
            return false;

        config_.type        = HistogramConfig::Type::Range;
        config_.num_bins    = 0;
        config_.sorted_bins = true;

        if (n < 1)
            return true;

        const Tinternal min_value_int = convertToInternal<Tinternal>(min_value);
        const Tinternal max_value_int = convertToInternal<Tinternal>(max_value);

        if (!histogram_helpers::checkFinite<Tinternal>(min_value_int) || 
            !histogram_helpers::checkFinite<Tinternal>(max_value_int) || 
            min_value_int >= max_value_int)
            return false;
        
        //compute suitable step size
        //@TODO: integer types should check if the number of integer values in the range is below n!
        Tinternal step_size_int = (max_value_int - min_value_int) / (Tinternal)n;
        step_size_ = convertFromInternal<Tinternal>(step_size_int);

        if (step_size_int == 0)
        {
            //should only happen on integer types when number of bins is bigger than range
            //in that case => reduce bins
            n             = (size_t)(max_value_int - min_value_int);
            step_size_int = 1;
            step_size_    = convertFromInternal<Tinternal>(step_size_int);
        }

        //should never happen
        traced_assert(n > 0);

        bins_.resize(n);
        ranges_.resize(n);

        config_.num_bins = n;

        //create bins
        for (size_t i = 0; i < n; ++i)
        {
            const T rmin = convertFromInternal<Tinternal>(min_value_int +  i      * step_size_int);
            const T rmax = convertFromInternal<Tinternal>(min_value_int + (i + 1) * step_size_int);

            bins_[ i ].min_value = rmin;
            bins_[ i ].max_value = rmax;

            //add to search structure
            ranges_[ i ] = rmin;
        }
        bins_.back().max_value    = max_value; //last bin includes everything up to max value
        bins_.back().max_included = true;      //maximum value is part of last bin

        //sort ranges
        std::sort(ranges_.begin(), ranges_.end());

        return true;
    }

    /**
     * Conversion of data to bin via stepsize and an internal computation type.
     */
    template<typename Tinternal = T>
    int binFromValueT(const T& v) const
    {
        //convert to internal type
        const Tinternal v_int   = convertToInternal<Tinternal>(v);
        const Tinternal min_int = convertToInternal<Tinternal>(bins_.front().min_value);
        const Tinternal ss_int  = convertToInternal<Tinternal>(step_size_.value());

        if (ss_int == 0)
            return -1;

        //round to bin
        return (int)((v_int - min_int) / ss_int);
    }

    /**
     * Conversion of data to bin via stepsize.
     */
    int binFromValue(const T& v) const
    {
        return binFromValueT<T>(v);
    }

    /**
     */
    std::pair<int,int> estimateTargetBinFromStepSize(const T& v, bool* ok) const
    {
        if (ok)
            *ok = false;

        if (!step_size_.has_value())
            return {-1, -1};

        if (bins_.empty())
            return {-1, -1};

        //not in bin range?
        if (v < bins_.front().min_value || v > bins_.back().max_value)
            return {-1, -1};
        
        //use step size to obtain index
        int idx_step = binFromValue(v);
        if (idx_step < 0)
            return {-1, -1};

        //add interval of 1 to be on the safe side (should not increase runtime by much)
        int n        = (int)bins_.size();
        int idx      = std::min(n - 1, idx_step);
        int idx_min  = std::max(0    , idx - 1);
        int idx_max  = std::min(n - 1, idx + 1);

        if (ok)
            *ok = true;

        return { idx_min, idx_max };
    }

    /**
     */
    std::pair<int,int> estimateTargetBinFromRanges(const T& v, bool* ok) const
    {
        if (ok)
            *ok = false;

        if (ranges_.empty() || ranges_.size() != bins_.size())
            return {-1, -1};

        //not in bin range?
        if (v < bins_.front().min_value || v > bins_.back().max_value)
            return {-1, -1};

        //use sorted ranges to obtain index
        auto it = std::upper_bound(ranges_.begin(), ranges_.end(), v);

        //add interval of 1 to be on the safe side (should not increase runtime by much)
        int n       = (int)ranges_.size();
        int idx     = it == ranges_.end() ? n - 1 : it - ranges_.begin();
        int idx_min = std::max(0    , idx - 1);
        int idx_max = std::min(n - 1, idx + 1);

        if (ok)
            *ok = true;

        return { idx_min, idx_max };
    }

    std::pair<int,int> estimateTargetBinFromCategories(const T& v, bool* ok) const
    {
        if (ok)
            *ok = false;

        if (categories_.empty() || categories_.size() != bins_.size())
            return {-1, -1};

        //in category mode the result cannot be "badly estimated", it's either there or not
        if (ok)
            *ok = true;

        //use category map to obtain index
        auto it = categories_.find(v);

        //found range?
        if (it != categories_.end())
            return { it->second, it->second };
        
        return {-1, -1};
    }

    /**
     * Checks if the value is in histogram range.
     */
    bool isInRange(const T& v) const
    {
        //categories might not even be sorted, so this check makes no sense
        if (config_.type == HistogramConfig::Type::Category)
            return true;

        return (v >= bins_.front().min_value && v <= bins_.back().max_value);
    }

    /**
     * Estimates a bin range the given value might be part of.
     */
    std::pair<int,int> estimateTargetBin(const T& v) const
    {
        if (bins_.empty())
            return {-1, -1};

        //not in bin range?
        if (!isInRange(v))
            return {-1, -1};

        //try different methods to estimate bin quickly, depending on what clues are available
        //note: some version of data type T might not support certain methods
        {
            bool ok;
            auto target_bin = estimateTargetBinFromStepSize(v, &ok);
            if (ok)
                return target_bin;
        }
        {
            bool ok;
            auto target_bin = estimateTargetBinFromRanges(v, &ok);
            if (ok)
                return target_bin;
        }
        {
            bool ok;
            auto target_bin = estimateTargetBinFromCategories(v, &ok);
            if (ok)
                return target_bin;
        }

        //last resort: search whole range
        return std::make_pair(0, (int)bins_.size() - 1);
    }

    /**
     * Find bin via linear search.
     * Returns the bin index if found, -1 otherwise.
     */
    int findBinLinear(const T& v, 
                      int* bin_min = nullptr, 
                      int* bin_max = nullptr) const
    {
        if (bins_.empty())
            return -1;

        //bins to search
        int bmin = bin_min ? *bin_min : 0;
        int bmax = bin_max ? *bin_max : bins_.size() - 1;

        for (int i = bmin; i <= bmax; ++i)
        {
            auto& b = bins_[ i ];

            //bin found?
            if (b.isInside(v))
                return i;
        }

        //no bin found
        return -1;
    }

    HistogramConfig               config_;
    std::vector<HistogramBinT<T>> bins_;
    std::map<T,int>               categories_;
    std::vector<T>                ranges_;
    boost::optional<T>            step_size_;
    HistogramBinT<T>              not_found_bin_;
};

/**
 * Prevent creation of histogram from data range for some types.
 */
template<>
inline bool HistogramT<std::string>::createFromRange(size_t n, const std::string& min_value, const std::string& max_value)
{
    throw std::runtime_error("HistogramT<T>::createFromRange: not implemented for data type std::string");
    return false;
}
template<>
inline bool HistogramT<bool>::createFromRange(size_t n, const bool& min_value, const bool& max_value)
{
    throw std::runtime_error("HistogramT<T>::createFromRange: not implemented for data type bool");
    return false;
}

/**
 * Prevent certain kinds of bin estimation for some types.
 */
template<>
inline std::pair<int,int> HistogramT<std::string>::estimateTargetBinFromStepSize(const std::string& v, bool* ok) const
{
    if (ok) *ok = false;
    return { -1, -1};
}
template<>
inline std::pair<int,int> HistogramT<bool>::estimateTargetBinFromStepSize(const bool& v, bool* ok) const
{
    if (ok) *ok = false;
    return { -1, -1};
}
template<>
inline std::pair<int,int> HistogramT<std::string>::estimateTargetBinFromRanges(const std::string& v, bool* ok) const
{
    if (ok) *ok = false;
    return { -1, -1};
}
template<>
inline std::pair<int,int> HistogramT<bool>::estimateTargetBinFromRanges(const bool& v, bool* ok) const
{
    if (ok) *ok = false;
    return { -1, -1};
}

/**
 * Conversion to and from an internal computation unit.
 */
template<>
template<>
inline long HistogramT<boost::posix_time::ptime>::convertToInternal<long>(const boost::posix_time::ptime& v) const
{
    const long v_l = Utils::Time::toLong(v);
    return v_l;
}
template<>
template<>
inline boost::posix_time::ptime HistogramT<boost::posix_time::ptime>::convertFromInternal<long>(const long& v) const
{
    const auto v_pt = Utils::Time::fromLong(v);
    return v_pt;
}
template<>
inline bool HistogramT<boost::posix_time::ptime>::createFromRange(size_t n, 
                                                                  const boost::posix_time::ptime& min_value, 
                                                                  const boost::posix_time::ptime& max_value)
{
    //compute using conversion to long
    return createFromRangeT<long>(n, min_value, max_value);
}

/**
 * Computation of bin from value and step size.
 */
template<>
inline int HistogramT<bool>::binFromValue(const bool& v) const
{
    //not possible
    return -1;
}
template<>
inline int HistogramT<std::string>::binFromValue(const std::string& v) const
{
    //not possible
    return -1;
}
template<>
inline int HistogramT<boost::posix_time::ptime>::binFromValue(const boost::posix_time::ptime& v) const
{
    //compute using conversion to long
    return binFromValueT<long>(v);
}
