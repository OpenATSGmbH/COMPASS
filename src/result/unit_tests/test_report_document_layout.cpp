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

#include "catch.hpp"

#include "report/reportdocumentlayout.h"
#include "report/reportdocumentview.h"
#include "report/section.h"

using ResultReport::ReportDocumentLayout;
using ResultReport::SectionContentType;

namespace
{
    ReportDocumentLayout::ContentCounts counts(unsigned int figures, unsigned int tables, unsigned int texts)
    {
        ReportDocumentLayout::ContentCounts c {};
        c[ (size_t)SectionContentType::Figure ] = figures;
        c[ (size_t)SectionContentType::Table  ] = tables;
        c[ (size_t)SectionContentType::Text   ] = texts;
        return c;
    }

    // four sections without contents, each one heading high
    ReportDocumentLayout headingsOnly(int heading_height)
    {
        ReportDocumentLayout layout;
        layout.setHeadingHeight(heading_height);

        for (int i = 0; i < 4; ++i)
            layout.addEntry(nullptr, 1, counts(0, 0, 0));

        return layout;
    }
}

TEST_CASE("ReportDocumentLayout - offsets of estimated entries", "[ReportDocumentLayout]")
{
    ReportDocumentLayout layout;
    layout.setHeadingHeight(30);

    layout.addEntry(nullptr, 0, counts(0, 0, 0));
    layout.addEntry(nullptr, 1, counts(1, 0, 0));
    layout.addEntry(nullptr, 1, counts(0, 2, 1));

    const int fig  = layout.contentHeight(SectionContentType::Figure);
    const int tab  = layout.contentHeight(SectionContentType::Table);
    const int text = layout.contentHeight(SectionContentType::Text);

    REQUIRE(layout.size() == 3);
    REQUIRE(layout.offsetOf(0) == 0);
    REQUIRE(layout.offsetOf(1) == 30);
    REQUIRE(layout.offsetOf(2) == 30 + 30 + fig);
    REQUIRE(layout.totalHeight() == 30 + 30 + fig + 30 + 2 * tab + text);
}

TEST_CASE("ReportDocumentLayout - indexAt at entry boundaries", "[ReportDocumentLayout]")
{
    auto layout = headingsOnly(50);

    REQUIRE(layout.indexAt(-10) == 0);
    REQUIRE(layout.indexAt(0) == 0);
    REQUIRE(layout.indexAt(49) == 0);
    REQUIRE(layout.indexAt(50) == 1);
    REQUIRE(layout.indexAt(149) == 2);
    REQUIRE(layout.indexAt(150) == 3);
    REQUIRE(layout.indexAt(199) == 3);
    REQUIRE(layout.indexAt(200) == 3);
    REQUIRE(layout.indexAt(100000) == 3);
}

TEST_CASE("ReportDocumentLayout - a measured height moves only the later entries", "[ReportDocumentLayout]")
{
    auto layout = headingsOnly(50);

    REQUIRE(layout.totalHeight() == 200);

    layout.setMeasuredHeight(1, 120);

    REQUIRE(layout.heightOf(1) == 120);
    REQUIRE(layout.offsetOf(0) == 0);
    REQUIRE(layout.offsetOf(1) == 50);
    REQUIRE(layout.offsetOf(2) == 170);
    REQUIRE(layout.offsetOf(3) == 220);
    REQUIRE(layout.totalHeight() == 270);
    REQUIRE(layout.indexAt(169) == 1);
    REQUIRE(layout.indexAt(170) == 2);

    SECTION("a new measurement replaces the old one")
    {
        layout.setMeasuredHeight(1, 60);

        REQUIRE(layout.offsetOf(2) == 110);
        REQUIRE(layout.totalHeight() == 210);
    }

    SECTION("cleared measurements fall back to the estimates")
    {
        layout.clearMeasuredHeights();

        REQUIRE(layout.offsetOf(2) == 100);
        REQUIRE(layout.totalHeight() == 200);
    }
}

TEST_CASE("ReportDocumentLayout - estimates follow the measured mean per content type", "[ReportDocumentLayout]")
{
    const int heading = 40;

    ReportDocumentLayout layout;
    layout.setHeadingHeight(heading);

    layout.addEntry(nullptr, 1, counts(1, 0, 0));
    layout.addEntry(nullptr, 1, counts(1, 0, 0));
    layout.addEntry(nullptr, 1, counts(1, 0, 0));
    layout.addEntry(nullptr, 1, counts(0, 1, 0));

    const int table_default = layout.contentHeight(SectionContentType::Table);

    layout.setMeasuredHeight(0, heading + 300);

    REQUIRE(layout.contentHeight(SectionContentType::Figure) == 300);
    REQUIRE(layout.heightOf(2) == heading + 300);
    REQUIRE(layout.contentHeight(SectionContentType::Table) == table_default);

    layout.setMeasuredHeight(1, heading + 500);

    REQUIRE(layout.contentHeight(SectionContentType::Figure) == 400);
    REQUIRE(layout.heightOf(2) == heading + 400);

    SECTION("a measured entry counts once")
    {
        layout.setMeasuredHeight(0, heading + 700);

        REQUIRE(layout.contentHeight(SectionContentType::Figure) == 600);
    }

    SECTION("estimates of unmeasured entries move the total")
    {
        REQUIRE(layout.totalHeight() == (heading + 300) + (heading + 500) + (heading + 400) + (heading + table_default));
    }
}

TEST_CASE("ReportDocumentLayout - mixed contents split the measured height by the estimates", "[ReportDocumentLayout]")
{
    ReportDocumentLayout layout;
    layout.setHeadingHeight(0);

    layout.addEntry(nullptr, 1, counts(1, 1, 0));

    const int fig = layout.contentHeight(SectionContentType::Figure);
    const int tab = layout.contentHeight(SectionContentType::Table);

    layout.setMeasuredHeight(0, 2 * (fig + tab));

    REQUIRE(layout.contentHeight(SectionContentType::Figure) == 2 * fig);
    REQUIRE(layout.contentHeight(SectionContentType::Table) == 2 * tab);
}

TEST_CASE("ReportDocumentLayout - numbers as in the LaTeX export", "[ReportDocumentLayout]")
{
    ResultReport::Section root("Results", "", nullptr, nullptr);

    auto& a   = root.addSubSection("A");
    auto& a1  = a.addSubSection("A1");
    auto& a1x = a1.addSubSection("A1x");
    auto& a2  = a.addSubSection("A2");
    auto& b   = root.addSubSection("B");
    auto& b1  = b.addSubSection("B1");
    auto& c   = root.addSubSection("C");

    REQUIRE(ReportDocumentLayout::sectionNumber(root) == "");
    REQUIRE(ReportDocumentLayout::sectionNumber(a)    == "1");
    REQUIRE(ReportDocumentLayout::sectionNumber(a1)   == "1.1");
    REQUIRE(ReportDocumentLayout::sectionNumber(a1x)  == "");
    REQUIRE(ReportDocumentLayout::sectionNumber(a2)   == "1.2");
    REQUIRE(ReportDocumentLayout::sectionNumber(b)    == "2");
    REQUIRE(ReportDocumentLayout::sectionNumber(b1)   == "2.1");
    REQUIRE(ReportDocumentLayout::sectionNumber(c)    == "3");

    ReportDocumentLayout layout;
    layout.setRootSection(&root, { &a });

    REQUIRE(layout.entry(layout.indexOf(&b1).value()).number == "2.1");
    REQUIRE(layout.entry(layout.indexOf(&c).value()).number  == "3");
}

TEST_CASE("ReportDocumentLayout - build order", "[ReportDocumentLayout]")
{
    // entries 0..9, each 100 high
    auto layout = headingsOnly(100);
    for (int i = 0; i < 6; ++i)
        layout.addEntry(nullptr, 1, counts(0, 0, 0));

    SECTION("entries in the area top to bottom, then the margin entries nearest first")
    {
        auto order = layout.buildOrder(450, 650, 200);

        REQUIRE(order == std::vector<size_t>({ 4, 5, 6, 3, 7, 2, 8 }));
    }

    SECTION("an entry which ends at the area top is outside of it")
    {
        auto order = layout.buildOrder(400, 500, 100);

        REQUIRE(order == std::vector<size_t>({ 4, 3, 5 }));
    }

    SECTION("clamped at the document ends")
    {
        REQUIRE(layout.buildOrder(0, 100, 150) == std::vector<size_t>({ 0, 1, 2 }));
        REQUIRE(layout.buildOrder(900, 1000, 100) == std::vector<size_t>({ 9, 8 }));
    }
}

TEST_CASE("ReportDocumentLayout - sections in tree order", "[ReportDocumentLayout]")
{
    ResultReport::Section root("Results", "", nullptr, nullptr);

    auto& a = root.addSubSection("A");
    auto& a1 = a.addSubSection("A1");
    auto& b = root.addSubSection("B");

    ReportDocumentLayout layout;
    layout.setRootSection(&root);

    REQUIRE(layout.size() == 4);
    REQUIRE(layout.entry(0).section == &root);
    REQUIRE(layout.entry(1).section == &a);
    REQUIRE(layout.entry(2).section == &a1);
    REQUIRE(layout.entry(3).section == &b);

    REQUIRE(layout.entry(2).depth == 2);
    REQUIRE(layout.entry(2).number == "1.1");
    REQUIRE(layout.entry(3).number == "2");

    REQUIRE(layout.indexOf(&b).value() == 3);
    REQUIRE(!layout.indexOf(nullptr).has_value());

    layout.setRootSection(nullptr);

    REQUIRE(layout.empty());
}

TEST_CASE("ReportDocumentLayout - collapsed sections", "[ReportDocumentLayout]")
{
    ResultReport::Section root("Results", "", nullptr, nullptr);

    auto& a  = root.addSubSection("A");
    auto& a1 = a.addSubSection("A1");
    auto& a2 = a.addSubSection("A2");
    auto& b  = root.addSubSection("B");
    auto& b1 = b.addSubSection("B1");

    ReportDocumentLayout layout;
    layout.setHeadingHeight(50);
    layout.setRootSection(&root, { &a });

    REQUIRE(layout.size() == 4);
    REQUIRE(layout.entry(1).section == &a);
    REQUIRE(layout.entry(2).section == &b);
    REQUIRE(layout.entry(2).number == "2");
    REQUIRE(layout.entry(3).section == &b1);
    REQUIRE(layout.entry(3).number == "2.1");
    REQUIRE(!layout.indexOf(&a1).has_value());

    layout.setMeasuredHeight(1, 70);
    layout.setMeasuredHeight(3, 90);

    SECTION("an expanded section brings back its sub-sections, the measured heights stay")
    {
        layout.setCollapsedSections({});

        REQUIRE(layout.size() == 6);
        REQUIRE(layout.entry(2).section == &a1);
        REQUIRE(layout.entry(3).section == &a2);
        REQUIRE(layout.entry(3).number == "1.2");
        REQUIRE(layout.entry(5).number == "2.1");

        REQUIRE(layout.heightOf(layout.indexOf(&a).value()) == 70);
        REQUIRE(layout.heightOf(layout.indexOf(&b1).value()) == 90);
        REQUIRE(!layout.entry(2).measured_height.has_value());
        REQUIRE(layout.totalHeight() == 50 + 70 + 50 + 50 + 50 + 90);
    }

    SECTION("a collapsed section hides its sub-sections")
    {
        layout.setCollapsedSections({ &a, &b });

        REQUIRE(layout.size() == 3);
        REQUIRE(layout.entry(2).section == &b);
        REQUIRE(!layout.indexOf(&b1).has_value());
        REQUIRE(layout.heightOf(1) == 70);
        REQUIRE(layout.totalHeight() == 50 + 70 + 50);
    }

    SECTION("a collapsed root holds only itself")
    {
        layout.setCollapsedSections({ &root });

        REQUIRE(layout.size() == 1);
        REQUIRE(layout.entry(0).section == &root);
    }
}

TEST_CASE("ReportDocumentView - wheel passes from an inner scroll area", "[ReportDocumentLayout]")
{
    using ResultReport::ReportDocumentView;

    // a positive vertical delta scrolls up
    REQUIRE(ReportDocumentView::innerAreaPassesWheel(0, 0, 100, 0, 120));
    REQUIRE(!ReportDocumentView::innerAreaPassesWheel(0, 0, 100, 0, -120));
    REQUIRE(ReportDocumentView::innerAreaPassesWheel(100, 0, 100, 0, -120));
    REQUIRE(!ReportDocumentView::innerAreaPassesWheel(100, 0, 100, 0, 120));
    REQUIRE(!ReportDocumentView::innerAreaPassesWheel(50, 0, 100, 0, 120));
    REQUIRE(!ReportDocumentView::innerAreaPassesWheel(50, 0, 100, 0, -120));

    // without vertical range
    REQUIRE(ReportDocumentView::innerAreaPassesWheel(0, 0, 0, 0, 120));
    REQUIRE(ReportDocumentView::innerAreaPassesWheel(0, 0, 0, 0, -120));

    // horizontal wheel stays with the inner area
    REQUIRE(!ReportDocumentView::innerAreaPassesWheel(0, 0, 0, 120, 0));
    REQUIRE(!ReportDocumentView::innerAreaPassesWheel(0, 0, 0, 120, 30));
}
