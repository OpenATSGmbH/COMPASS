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

#include "report/sectioncontenttable.h"

#include <QStandardItemModel>

using namespace ResultReport;

TEST_CASE("Report table Show Unused hides inactive rows", "[report_table]")
{
    QStandardItemModel source(3, 2);

    auto setRowStyle = [&] (int row, unsigned int style)
    {
        source.setData(source.index(row, 0), style, SectionContentTable::CellStyleRole);
    };

    setRowStyle(0, 0u);
    setRowStyle(1, CellStyleInactive);
    setRowStyle(2, CellStyleTextBold);

    TableQSortFilterProxyModel proxy;
    proxy.setSourceModel(&source);

    SECTION("all rows are shown by default")
    {
        CHECK(proxy.showUnused());
        CHECK(proxy.rowCount() == 3);
    }

    SECTION("inactive rows are hidden when Show Unused is off")
    {
        proxy.showUnused(false);
        proxy.invalidate();

        REQUIRE(proxy.rowCount() == 2);
        CHECK(proxy.mapToSource(proxy.index(0, 0)).row() == 0);
        CHECK(proxy.mapToSource(proxy.index(1, 0)).row() == 2);
    }
}
