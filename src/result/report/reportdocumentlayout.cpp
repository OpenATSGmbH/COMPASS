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

#include "report/reportdocumentlayout.h"
#include "report/section.h"

#include "traced_assert.h"

#include <algorithm>
#include <cmath>

namespace ResultReport
{

const int ReportDocumentLayout::DefaultHeadingHeight = 40;

// indexed by SectionContentType: figure, table, text
const std::array<int, ReportDocumentLayout::NumContentTypes> ReportDocumentLayout::DefaultContentHeights = { 500, 300, 100 };

// the LaTeX export numbers \section and \subsection (secnumdepth 2 of the report class)
const int ReportDocumentLayout::NumberedDepth = 2;

/**
 */
ReportDocumentLayout::ReportDocumentLayout() = default;

/**
 * Number of the section as in the LaTeX export, e.g. "2.3", from its position in the full tree, so
 * collapsed sections do not change it. Empty for the root section and below the numbered depths.
 */
std::string ReportDocumentLayout::sectionNumber(const Section& section)
{
    std::vector<int> rows;
    for (const Section* s = &section; s->parentSection(); s = s->parentSection())
    {
        if ((int)rows.size() == NumberedDepth)
            return "";

        rows.push_back(s->row() + 1);
    }

    std::string number;
    for (auto it = rows.rbegin(); it != rows.rend(); ++it)
        number += (number.empty() ? "" : ".") + std::to_string(*it);

    return number;
}

/**
 * Builds the entries of the section and its sub-sections in tree order (depth first),
 * without the sub-sections of the collapsed sections.
 */
void ReportDocumentLayout::setRootSection(Section* root, const SectionSet& collapsed)
{
    clear();

    root_ = root;

    addSections(collapsed);
}

/**
 * Builds the entries again with other collapsed sections. The measured heights of the
 * sections which stay in the document are kept.
 */
void ReportDocumentLayout::setCollapsedSections(const SectionSet& collapsed)
{
    std::vector<Entry> old_entries;
    old_entries.swap(entries_);

    indexes_.clear();
    offsets_valid_ = false;

    addSections(collapsed);

    for (auto& old : old_entries)
    {
        auto index = indexOf(old.section);

        if (!index.has_value())
        {
            //an entry measured again later must not count twice
            removeMeasuredShare(old);
            continue;
        }

        auto& e = entries_[ index.value() ];

        e.measured_height        = old.measured_height;
        e.measured_share         = old.measured_share;
        e.measured_share_counted = old.measured_share_counted;
    }
}

/**
 */
void ReportDocumentLayout::addSections(const SectionSet& collapsed)
{
    if (!root_)
        return;

    std::vector<std::pair<Section*, int>> stack = { { root_, 0 } };

    while (!stack.empty())
    {
        auto [ section, depth ] = stack.back();
        stack.pop_back();

        ContentCounts counts {};
        counts[ (size_t)SectionContentType::Figure ] = section->numFigures();
        counts[ (size_t)SectionContentType::Table  ] = section->numTables();
        counts[ (size_t)SectionContentType::Text   ] = section->numTexts();

        addEntry(section, depth, counts);

        if (collapsed.count(section))
            continue;

        auto subs = section->subSections(false);
        for (auto it = subs.rbegin(); it != subs.rend(); ++it)
            stack.emplace_back(it->get(), depth + 1);
    }
}

/**
 * Appends an entry. Entries must be added in tree order.
 */
void ReportDocumentLayout::addEntry(Section* section, int depth, const ContentCounts& num_contents)
{
    Entry e;
    e.section      = section;
    e.depth        = depth;
    e.num_contents = num_contents;

    if (section)
    {
        e.number = sectionNumber(*section);
        indexes_[ section ] = entries_.size();
    }

    entries_.push_back(e);

    offsets_valid_ = false;
}

/**
 */
void ReportDocumentLayout::clear()
{
    root_ = nullptr;

    entries_.clear();
    indexes_.clear();

    measured_sums_   = {};
    measured_counts_ = {};

    offsets_valid_ = false;
}

/**
 */
Section* ReportDocumentLayout::rootSection() const
{
    return root_;
}

/**
 */
size_t ReportDocumentLayout::size() const
{
    return entries_.size();
}

/**
 */
bool ReportDocumentLayout::empty() const
{
    return entries_.empty();
}

/**
 */
const ReportDocumentLayout::Entry& ReportDocumentLayout::entry(size_t index) const
{
    return entries_.at(index);
}

/**
 */
std::optional<size_t> ReportDocumentLayout::indexOf(const Section* section) const
{
    auto it = indexes_.find(section);
    if (it == indexes_.end())
        return {};

    return it->second;
}

/**
 */
int ReportDocumentLayout::totalHeight() const
{
    updateOffsets();
    return offsets_.back();
}

/**
 */
int ReportDocumentLayout::offsetOf(size_t index) const
{
    traced_assert(index < entries_.size());

    updateOffsets();
    return offsets_[ index ];
}

/**
 */
int ReportDocumentLayout::heightOf(size_t index) const
{
    const auto& e = entries_.at(index);

    return e.measured_height.has_value() ? e.measured_height.value() : estimatedHeight(e);
}

/**
 * Index of the entry which covers the given position, clamped to the first and last entry.
 */
size_t ReportDocumentLayout::indexAt(int y) const
{
    traced_assert(!entries_.empty());

    updateOffsets();

    auto end = offsets_.begin() + entries_.size();
    auto it  = std::upper_bound(offsets_.begin(), end, y);

    if (it == offsets_.begin())
        return 0;

    return (size_t)(it - offsets_.begin()) - 1;
}

/**
 * Indexes of the entries which overlap the area from top to bottom, extended by the margin above and
 * below, in build order: first the entries in the area from top to bottom, then the others by their
 * distance to the area.
 */
std::vector<size_t> ReportDocumentLayout::buildOrder(int top, int bottom, int margin) const
{
    if (entries_.empty())
        return {};

    const size_t first = indexAt(std::max(0, top - margin));
    const size_t last  = indexAt(std::max(0, bottom + margin - 1));

    std::vector<std::pair<int, size_t>> order; // distance to the area, index

    for (size_t i = first; i <= last; ++i)
    {
        const int entry_top    = offsetOf(i);
        const int entry_bottom = entry_top + heightOf(i);

        int distance = 0;
        if (entry_bottom <= top)
            distance = top - entry_bottom + 1;
        else if (entry_top >= bottom)
            distance = entry_top - bottom + 1;

        order.emplace_back(distance, i);
    }

    std::sort(order.begin(), order.end());

    std::vector<size_t> indexes;
    for (const auto& o : order)
        indexes.push_back(o.second);

    return indexes;
}

/**
 * Sets the measured height and moves the estimated height per content type to the mean of the measured
 * heights. The content height of the entry is split over its contents in proportion to their current
 * estimates.
 */
void ReportDocumentLayout::setMeasuredHeight(size_t index, int height)
{
    auto& e = entries_.at(index);

    removeMeasuredShare(e);

    e.measured_height = height;
    offsets_valid_    = false;

    unsigned int num_contents = 0;
    double       weight_sum   = 0.0;

    for (size_t t = 0; t < NumContentTypes; ++t)
    {
        num_contents += e.num_contents[ t ];
        weight_sum   += e.num_contents[ t ] * (double)contentHeight((SectionContentType)t);
    }

    if (num_contents == 0)
        return;

    const double content_height = std::max(0, height - heading_height_);

    for (size_t t = 0; t < NumContentTypes; ++t)
    {
        if (e.num_contents[ t ] == 0)
            continue;

        const double weight = weight_sum > 0.0 ? e.num_contents[ t ] * (double)contentHeight((SectionContentType)t) / weight_sum
                                               : e.num_contents[ t ] / (double)num_contents;

        e.measured_share[ t ]  = content_height * weight;
        measured_sums_[ t ]   += e.measured_share[ t ];
        measured_counts_[ t ] += e.num_contents[ t ];
    }

    e.measured_share_counted = true;
}

/**
 */
void ReportDocumentLayout::removeMeasuredShare(Entry& entry)
{
    if (!entry.measured_share_counted)
        return;

    for (size_t t = 0; t < NumContentTypes; ++t)
    {
        measured_sums_[ t ]   -= entry.measured_share[ t ];
        measured_counts_[ t ] -= entry.num_contents[ t ];
    }

    entry.measured_share         = {};
    entry.measured_share_counted = false;
}

/**
 * Removes the measured heights (e.g. after a width change). The content type statistics stay.
 */
void ReportDocumentLayout::clearMeasuredHeights()
{
    for (auto& e : entries_)
        e.measured_height.reset();

    offsets_valid_ = false;
}

/**
 */
void ReportDocumentLayout::setHeadingHeight(int height)
{
    heading_height_ = height;
    offsets_valid_  = false;
}

/**
 */
int ReportDocumentLayout::headingHeight() const
{
    return heading_height_;
}

/**
 * Estimated height of one content of the given type.
 */
int ReportDocumentLayout::contentHeight(SectionContentType type) const
{
    size_t t = (size_t)type;
    traced_assert(t < NumContentTypes);

    if (measured_counts_[ t ] == 0)
        return DefaultContentHeights[ t ];

    return (int)std::lround(measured_sums_[ t ] / measured_counts_[ t ]);
}

/**
 */
int ReportDocumentLayout::estimatedHeight(const Entry& entry) const
{
    int h = heading_height_;

    for (size_t t = 0; t < NumContentTypes; ++t)
        if (entry.num_contents[ t ] > 0)
            h += (int)entry.num_contents[ t ] * contentHeight((SectionContentType)t);

    return h;
}

/**
 */
void ReportDocumentLayout::updateOffsets() const
{
    if (offsets_valid_)
        return;

    offsets_.resize(entries_.size() + 1);
    offsets_[ 0 ] = 0;

    for (size_t i = 0; i < entries_.size(); ++i)
        offsets_[ i + 1 ] = offsets_[ i ] + heightOf(i);

    offsets_valid_ = true;
}

}
