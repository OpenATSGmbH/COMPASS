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

#include "report/reportdefs.h"

#include <array>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ResultReport
{

class Section;

/**
 * Vertical layout of all report sections as one continuous document, without widgets.
 * The entries are in tree order (depth first). An entry has a measured height once its
 * widget was built, else an estimated height from the number of its contents per type.
 * A collapsed section has an entry, its sub-sections have none.
 */
class ReportDocumentLayout
{
public:
    static constexpr size_t NumContentTypes = 3;

    typedef std::array<unsigned int, NumContentTypes> ContentCounts;
    typedef std::unordered_set<const Section*>        SectionSet;

    struct Entry
    {
        Section*           section = nullptr;
        int                depth   = 0;
        std::string        number;
        ContentCounts      num_contents {};
        std::optional<int> measured_height;

        // share of the measured height in the content type statistics, removed on a new measurement
        std::array<double, NumContentTypes> measured_share {};
        bool                                measured_share_counted = false;
    };

    ReportDocumentLayout();

    void setRootSection(Section* root, const SectionSet& collapsed = {});
    void setCollapsedSections(const SectionSet& collapsed);
    void addEntry(Section* section, int depth, const ContentCounts& num_contents);
    void clear();

    Section* rootSection() const;

    size_t size() const;
    bool empty() const;
    const Entry& entry(size_t index) const;
    std::optional<size_t> indexOf(const Section* section) const;

    int totalHeight() const;
    int offsetOf(size_t index) const;
    int heightOf(size_t index) const;
    size_t indexAt(int y) const;
    std::vector<size_t> buildOrder(int top, int bottom, int margin) const;

    void setMeasuredHeight(size_t index, int height);
    void clearMeasuredHeights();

    void setHeadingHeight(int height);
    int headingHeight() const;
    int contentHeight(SectionContentType type) const;

    static const int DefaultHeadingHeight;
    static const std::array<int, NumContentTypes> DefaultContentHeights;

    static const int NumberedDepth;

    static std::string sectionNumber(const Section& section);

private:
    void addSections(const SectionSet& collapsed);
    void removeMeasuredShare(Entry& entry);
    int estimatedHeight(const Entry& entry) const;
    void updateOffsets() const;

    Section*                                    root_ = nullptr;
    std::vector<Entry>                          entries_;
    std::unordered_map<const Section*, size_t> indexes_;

    int heading_height_ = DefaultHeadingHeight;

    std::array<double, NumContentTypes>       measured_sums_ {};
    std::array<unsigned int, NumContentTypes> measured_counts_ {};

    mutable std::vector<int> offsets_; // size()+1 entries, last one is the total height
    mutable bool             offsets_valid_ = false;
};

}
