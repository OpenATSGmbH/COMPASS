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

#include "histogram.h"
#include "histogramgenerator.h"
#include "histograminitializer.h"
#include "dbcontent.h"

#include <functional>
#include <set>
#include <string>

namespace dbContent
{
    class Variable;
    class MetaVariable;
}

/**
 * Buffer data based histogram generator.
 * Completely hides the concrete buffer data type.
 */
class HistogramGeneratorBuffer : public HistogramGenerator
{
public:
    typedef std::map<std::string, std::shared_ptr<Buffer>> Data;

    /// Per-row allow predicate. If set, rows for which the predicate returns
    /// false are skipped both during min/max scanning and during bin counting.
    /// Called as (dbcontent_name, row_index). An unset filter admits every row.
    typedef std::function<bool(const std::string&, unsigned int)> RowFilter;

    /// Per-row layer id lookup. Returns the layer id
    /// ("<ds_type>:<ds_name>:L<n>:<dbcontent>") that a given row belongs to,
    /// or an empty string for unmappable rows (no ds_id/line_id). When set,
    /// the generator groups its per-layer result buckets by this id - one bar
    /// set per visible layer, stacked in the chart. Without it, the generator
    /// falls back to per-dbcontent grouping.
    typedef std::function<std::string(const std::string&, unsigned int)> RowLayerLookup;

    HistogramGeneratorBuffer(Data* buffer_data,
                             dbContent::Variable* variable,
                             dbContent::MetaVariable* meta_variable);
    virtual ~HistogramGeneratorBuffer() = default;

    virtual bool hasData() const override;

    bool dataNotInBuffer() const { return data_not_in_buffer_; }

    void setRowFilter(RowFilter f) { row_filter_ = std::move(f); }
    bool hasRowFilter() const { return static_cast<bool>(row_filter_); }
    bool rowAllowed(const std::string& dbc, unsigned int i) const
    {
        return !row_filter_ || row_filter_(dbc, i);
    }

    void setRowLayerLookup(RowLayerLookup f) { row_layer_lookup_ = std::move(f); }
    bool hasRowLayerLookup() const { return static_cast<bool>(row_layer_lookup_); }
    std::string rowLayerId(const std::string& dbc, unsigned int i) const
    {
        return row_layer_lookup_ ? row_layer_lookup_(dbc, i) : std::string{};
    }

    /// Group key for a row: layer id if a lookup is installed, otherwise the
    /// dbcontent name (legacy per-dbcontent grouping). Used as the key for
    /// `histograms_` / `intermediate_data_.content_data`.
    std::string groupKey(const std::string& dbc, unsigned int i) const
    {
        if (row_layer_lookup_)
            return row_layer_lookup_(dbc, i);
        return dbc;
    }

protected:
    dbContent::Variable* currentVariable(const std::string& db_content) const;
    Data* currentData() { return buffer_data_; }

    virtual bool select_impl(unsigned int bin0, unsigned int bin1) override;

    virtual bool selectBuffer(const std::string& db_content, 
                              Buffer& buffer,
                              unsigned int bin0, 
                              unsigned int bin1,
                              bool select_min_max,
                              bool select_null, 
                              bool add_to_selection) = 0;
    
    void setDataNotInBuffer(bool ok) { data_not_in_buffer_ = ok; }

private:
    Data*                    buffer_data_        = nullptr; //governed buffer data
    dbContent::Variable*     variable_           = nullptr; //governed variable
    dbContent::MetaVariable* meta_variable_      = nullptr; //governed meta-variable
    bool                     data_not_in_buffer_ = false;

    RowFilter                row_filter_;
    RowLayerLookup           row_layer_lookup_;
};

/**
 * Histogram generator specialized on the governed variables data type.
 */
template <typename T>
class HistogramGeneratorBufferT : public HistogramGeneratorBuffer
{
public:
    typedef std::map<std::string, HistogramT<T>> Histograms;

    HistogramGeneratorBufferT(Data* buffer_data, 
                              dbContent::Variable* variable,
                              dbContent::MetaVariable* meta_variable)
    :   HistogramGeneratorBuffer(buffer_data, variable, meta_variable) {}

    virtual ~HistogramGeneratorBufferT() = default;

protected:
    /**
     */
    void reset_impl() override final
    {
        resetInternal();
    }

    /**
     */
    virtual bool generateHistograms_impl() override final
    {
        if (!hasData())
            return false;

        resetInternal();

        std::vector<std::string> scanned_contents;

        //scan all buffers (min-max etc.)
        for (auto& elem : *currentData())
        {
            bool ok = scanBuffer(elem.first, *elem.second);

            if (ok)
                scanned_contents.push_back(elem.first);
            else
                logdbg << "could not scan buffer of DBContent " << elem.first;
        }

        //no data range available -> no good
        if (scanned_contents.empty() || !histogram_init_.valid())
            return false;

        auto range = histogram_init_.getRange();

        logdbg << "range: min = " << range->first << ", max = " << range->second;

        auto config = histogram_init_.generateConfiguration();

        logdbg << "config: num bins = " << config.num_bins << ", sorted = " << config.sorted_bins << ", type = " << (int)config.type;

        //remember a representative variable for bin label formatting
        if (!scanned_contents.empty())
            label_variable_ = currentVariable(scanned_contents.front());

        //enumerate the group keys that will contribute data - one bucket per
        //visible layer (when a row-layer lookup is installed) or per scanned
        //dbcontent (legacy fallback). Each bucket gets its own histogram copy
        //so zoom_impl and per-bucket bin counts can proceed independently.
        std::set<std::string> keys;
        if (hasRowLayerLookup())
        {
            for (auto& elem : *currentData())
            {
                const std::string& dbc = elem.first;
                const Buffer&      buf = *elem.second;
                const unsigned int n   = buf.size();
                for (unsigned int i = 0; i < n; ++i)
                {
                    if (!rowAllowed(dbc, i))
                        continue;
                    std::string lid = rowLayerId(dbc, i);
                    if (lid.empty())
                        continue;
                    keys.insert(lid);
                }
            }
        }
        else
        {
            for (const auto& dbc : scanned_contents)
                keys.insert(dbc);
        }

        for (const auto& key : keys)
        {
            auto& h = histograms_[key];
            histogram_init_.initHistogram(h, config);
        }

        return true;
    }

    /**
     */
    virtual bool refill_impl() override final
    {
        logdbg;

        //reinit intermediate data

        logdbg << "intermediate data";
        initIntermediateData();

        //add all buffers
        for (auto& elem : *currentData())
        {
            logdbg << "add buffer " << elem.first;

            bool ok = addBuffer(elem.first, *elem.second);

            if (!ok)
                logdbg << "could not add buffer of dbcontent " << elem.first;
        }

        intermediate_data_.buffer_nan_count  = histogram_init_.numNanValues();
        intermediate_data_.buffer_null_count = histogram_init_.numNullValues();

        logdbg << "done";

        return true;
    }

    /**
     */
    bool selectBuffer(const std::string& db_content,
                      Buffer& buffer,
                      unsigned int bin0,
                      unsigned int bin1,
                      bool select_min_max,
                      bool select_null,
                      bool add_to_selection) override final
    {
        //all histograms share the same config, so any one is sufficient for
        //bin lookup. bail if there are none (nothing selectable).
        if (histograms_.empty())
            return true;

        const auto& bin_lookup = histograms_.begin()->second;

        //variable not mapped to this dbcontent (normal for meta-vars that
        //don't apply to every dbcontent) - skip, not an error.
        auto variable = currentVariable(db_content);
        if (!variable)
            return true;

        std::string current_var_name = variable->name();

        //buffer genuinely missing the column - same treatment (skip).
        if (!buffer.has<T>(current_var_name))
            return true;

        NullableVector<T>& data = buffer.get<T>(current_var_name);

        unsigned int data_size = data.contentSize();

        //selected vector?
        traced_assert(buffer.has<bool>(dbcontent_vars::selected_var_.name()));
        NullableVector<bool>& selected_vec = buffer.get<bool>(dbcontent_vars::selected_var_.name());

        unsigned int select_cnt = 0;

        const bool has_filter = hasRowFilter();

        for (unsigned int cnt=0; cnt < data_size; ++cnt)
        {
            if (has_filter && !rowAllowed(db_content, cnt))
                continue;

            //check null case
            if (data.isNull(cnt))
            {
                if (select_null)
                {
                    selected_vec.set(cnt, true);
                    ++select_cnt;
                }
                else if (!add_to_selection)
                {
                    selected_vec.set(cnt, false);
                }
                continue;
            }

            if (!select_min_max)
            {
                //leave value "as is"
                if (!add_to_selection || selected_vec.isNull(cnt))
                    selected_vec.set(cnt, false);
                continue;
            }

            //find bin for data (any layer's histogram works, shared config)
            int bin_idx = bin_lookup.findBin(data.get(cnt));
            if (bin_idx < 0)
                continue;

            //bin inside selection range?
            bool select = ((unsigned int)bin_idx >= bin0 && (unsigned int)bin_idx <= bin1);
            if (!select && add_to_selection && !selected_vec.isNull(cnt))
                select = selected_vec.get(cnt);

            selected_vec.set(cnt, select);

            if (select)
                ++select_cnt;
        }

        loginf << "content = " << db_content << ", selected " << select_cnt;

        return true;
    }

    /**
     * Rearranges the bins to show the given subrange of bins.
     */
    bool zoom_impl(unsigned int bin0, unsigned int bin1) override final
    {
        if (!hasValidResult())
            return false;

        for (auto& elem : histograms_)
        {
            if (!elem.second.zoom(bin0, bin1))
                return false;
        }
        
        return true;
    }

private:
    /**
     * Resets all internal structures.
     */
    void resetInternal()
    {
        histograms_     = {};
        histogram_init_ = {};
        label_variable_ = nullptr;

        setDataNotInBuffer(false);
    }

    /**
     * Inits the intermediate data structures based on the current configuration.
     */
    void initIntermediateData()
    {
        logdbg;

        //reset existing histogram bins
        for (auto& elem : histograms_)
            elem.second.resetBins();

        intermediate_data_ = {};

        logdbg << "doing histograms";

        for (auto& elem : histograms_)
        {
            logdbg << "histograms " << elem.first;

            auto& d = intermediate_data_.content_data[ elem.first ];
            d.init(elem.second.numBins());

            logdbg << "bins";

            d.bins_are_sorted     = elem.second.configuration().sorted_bins;
            d.bins_are_categories = elem.second.configuration().type == HistogramConfig::Type::Category;

            logdbg << "labels";

            //generate labels
            for (size_t i = 0; i < elem.second.numBins(); ++i)
            {
                d.bin_data[ i ].labels = labelsForBin((int)i);
            }
        }

        logdbg << "done";
    }

    /**
     * Ask the histogram for a nice bin label.
     */
    BinLabels labelsForBin(int bin) const
    {
        logdbg << "bin " << bin;

        if (bin < 0 || histograms_.empty() || !label_variable_)
            return {};

        auto it = histograms_.begin();

        if (bin >= (int)it->second.numBins())
            return {};

        const auto& b = it->second.getBin(bin);

        BinLabels labels;
        labels.label     = b.label(label_variable_);
        labels.label_min = b.labelMin(label_variable_);
        labels.label_max = b.labelMax(label_variable_);

        return labels;
    }

    /**
     * Scans the buffer and extracts data needed for histogram generation.
     */
    bool scanBuffer(const std::string& db_content, Buffer& buffer)
    {
        auto variable = currentVariable(db_content);

        //variable not available for dbcontent
        if (!variable)
            return false;

        std::string current_var_name = variable->name();

        if (!buffer.has<T>(current_var_name))
        {
            //the variable should be part of the db content, but it is missing.
            //this hints that a reload is needed, so log it
            setDataNotInBuffer(true);
            return false;
        }

        NullableVector<T>& data = buffer.get<T>(current_var_name);

        //loginf << "Scanning buffer of dbc " << db_content << " size = " << data.size();

        if (hasRowFilter())
        {
            const std::string dbc = db_content;
            if (!histogram_init_.scanFiltered(data,
                    [this, dbc](size_t i) { return rowAllowed(dbc, (unsigned int)i); }))
                return false;
        }
        else
        {
            if (!histogram_init_.scan(data))
                return false;
        }

        return true;
    }

    /**
     * Add buffer content to histogram.
     */
    bool addBuffer(const std::string& db_content, Buffer& buffer)
    {
        auto variable = currentVariable(db_content);

        //no variable set?
        if (!variable)
            return false;

        //valid init happened?
        if (!histogram_init_.valid())
            return false;

        std::string current_var_name = variable->name();

        //buffer does not obtain variable?
        if (!buffer.has<T>(current_var_name))
            return false;

        NullableVector<T>& data = buffer.get<T>(current_var_name);
        unsigned int data_size = data.contentSize();

        traced_assert(buffer.has<bool>(dbcontent_vars::selected_var_.name()));
        NullableVector<bool>& selected_vec = buffer.get<bool>(dbcontent_vars::selected_var_.name());

        const bool has_filter        = hasRowFilter();
        const bool has_layer_lookup  = hasRowLayerLookup();

        //add variable content
        for (unsigned int cnt=0; cnt < data_size; ++cnt)
        {
            if (has_filter && !rowAllowed(db_content, cnt))
                continue;

            //resolve group key: layer id (per-layer mode) or dbcontent (legacy)
            const std::string key = has_layer_lookup
                                  ? rowLayerId(db_content, cnt)
                                  : db_content;
            if (key.empty())
                continue; //unmappable row

            auto hist_it = histograms_.find(key);
            if (hist_it == histograms_.end() || hist_it->second.numBins() < 1)
                continue;

            auto interm_it = intermediate_data_.content_data.find(key);
            if (interm_it == intermediate_data_.content_data.end())
                continue;

            auto& histogram   = hist_it->second;
            auto& interm_data = interm_it->second;

            bool selected = !selected_vec.isNull(cnt) && selected_vec.get(cnt);
            bool is_null  = data.isNull(cnt);

            //value null?
            if (is_null)
            {
                if (selected)
                    ++interm_data.null_selected_count;
                else
                    ++interm_data.null_count;
                continue;
            }

            //find bin
            int bin_idx = histogram.findBin(data.get(cnt));

            if (bin_idx < 0)   // is non-insertable?
            {
                ++interm_data.not_inserted_count;

                if (bin_idx == -2) //not finite
                    ++interm_data.nan_count;
            }
            else if (selected) // is selected?
            {
                ++interm_data.bin_data.at(bin_idx).selected;
            }
            else //just your typical valid-unselected-joe
            {
                ++interm_data.bin_data.at(bin_idx).count;
            }
        }

        return true;
    }

    HistogramInitializerT<T> histogram_init_;
    Histograms               histograms_;     //histograms per group key (layer id or dbcontent)
    dbContent::Variable*     label_variable_ = nullptr; //representative variable for bin labels
};
