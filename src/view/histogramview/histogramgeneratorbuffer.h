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

    HistogramGeneratorBuffer(Data* buffer_data, 
                             dbContent::Variable* variable,
                             dbContent::MetaVariable* meta_variable);
    virtual ~HistogramGeneratorBuffer() = default;

    virtual bool hasData() const override;

    bool dataNotInBuffer() const { return data_not_in_buffer_; }

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

        //init needed histograms
        for (const auto& db_content : scanned_contents)
        {
            auto& h = histograms_[ db_content ];

            //@TODO: needs further reaction on init fail?
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
        //no histogram yet for db content?
        auto it = histograms_.find(db_content);
        if (it == histograms_.end())
            return true;
        
        //no variable?
        auto variable = currentVariable(db_content);
        if (!variable)
            return false;

        std::string current_var_name = variable->name();

        //buffer does not obtain variable?
        if (!buffer.has<T>(current_var_name))
            return false;

        NullableVector<T>& data = buffer.get<T>(current_var_name);

        unsigned int data_size = data.contentSize();

        //selected vector?
        traced_assert(buffer.has<bool>(DBContent::selected_var.name()));
        NullableVector<bool>& selected_vec = buffer.get<bool>(DBContent::selected_var.name());

        unsigned int select_cnt = 0;

        for (unsigned int cnt=0; cnt < data_size; ++cnt)
        {
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

            //find bin for data
            int bin_idx = it->second.findBin(data.get(cnt));
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
        histograms_         = {};
        histogram_init_     = {};
        
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

        if (bin < 0 || histograms_.empty())
            return {};

        auto it = histograms_.begin();

        if (bin >= (int)it->second.numBins())
            return {};

        auto var = currentVariable(it->first);
        if (!var)
            return {};

        const auto& b = it->second.getBin(bin);

        BinLabels labels;
        labels.label     = b.label(var);
        labels.label_min = b.labelMin(var);
        labels.label_max = b.labelMax(var);

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
        
        if (!histogram_init_.scan(data))
            return false;

        return true;
    }

    /**
     * Add buffer content to histogram.
     */
    bool addBuffer(const std::string& db_content, Buffer& buffer)
    {
        //adding buffer needs previously initialized result and histogram for dbcontent type
        if (intermediate_data_.content_data.find(db_content) == intermediate_data_.content_data.end() ||
            histograms_.find(db_content) == histograms_.end())
            return false;

        auto variable = currentVariable(db_content);

        //no variable set?
        if (!variable)
            return false;

        //valid init happend?
        if (!histogram_init_.valid())
            return false;

        std::string current_var_name = variable->name();

        //buffer does not obtain variable?
        if (!buffer.has<T>(current_var_name))
            return false;

        NullableVector<T>& data = buffer.get<T>(current_var_name);
        unsigned int data_size = data.contentSize();

        traced_assert(buffer.has<bool>(DBContent::selected_var.name()));
        NullableVector<bool>& selected_vec = buffer.get<bool>(DBContent::selected_var.name());

        auto& histogram = histograms_[ db_content ];

        //histogram badly configured?
        if (histogram.numBins() < 1)
            return false;

        auto& interm_data = intermediate_data_.content_data[ db_content ];

        //add variable content
        for (unsigned int cnt=0; cnt < data_size; ++cnt)
        {
            bool selected = !selected_vec.isNull(cnt) && selected_vec.get(cnt);
            bool is_null  = data.isNull(cnt);

            //value null?
            if (is_null)
            {
                if (selected)
                {
                    ++interm_data.null_selected_count;
                }
                else 
                {
                    ++interm_data.null_count;
                }
                continue;
            }

            //find bin
            //@TODO: we use the histogram as a bin finder and store the counts externally,
            //but we could also add some "extra data" to each histogram bin in the future,
            //in order to track multiple per-bin counts inside the histogram itself.
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
    Histograms               histograms_;     //histograms per db content type
};
