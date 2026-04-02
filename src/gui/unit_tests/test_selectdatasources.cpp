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
#include "selectdatasourceswidget.h"

#include <QCheckBox>
#include <QSignalSpy>

// ─── Mock ───────────────────────────────────────────────────────────────────

class MockDataSourceProvider : public IDataSourceProvider
{
public:
    std::vector<DataSourceInfo> sources;

    std::vector<DataSourceInfo> dataSourceInfos() const override
    {
        return sources;
    }
};

// ─── Tests ──────────────────────────────────────────────────────────────────

TEST_CASE("SelectDataSourcesWidget empty provider", "[gui][selectds]")
{
    MockDataSourceProvider mock;

    SelectDataSourcesWidget widget(mock, "Test", "Radar");
    widget.updateSelected({});

    CHECK(widget.findChildren<QCheckBox*>().size() == 0);
}

TEST_CASE("SelectDataSourcesWidget filters by ds_type", "[gui][selectds]")
{
    MockDataSourceProvider mock;
    mock.sources = {
        {1, "Radar1", "Radar"},
        {2, "MLAT1",  "MLAT"},
        {3, "Radar2", "Radar"},
        {4, "ADSB1",  "ADSB"}
    };

    SelectDataSourcesWidget widget(mock, "Radars", "Radar");
    widget.updateSelected({});

    // Only the 2 Radar sources should have checkboxes
    CHECK(widget.findChildren<QCheckBox*>().size() == 2);
}

TEST_CASE("SelectDataSourcesWidget checkboxes default to selected", "[gui][selectds]")
{
    MockDataSourceProvider mock;
    mock.sources = {
        {10, "Sensor1", "Radar"},
        {20, "Sensor2", "Radar"}
    };

    SelectDataSourcesWidget widget(mock, "Radars", "Radar");
    widget.updateSelected({});  // empty selection → all auto-selected

    auto checkboxes = widget.findChildren<QCheckBox*>();
    for (auto* cb : checkboxes)
        CHECK(cb->isChecked());
}

TEST_CASE("SelectDataSourcesWidget respects selection map", "[gui][selectds]")
{
    MockDataSourceProvider mock;
    mock.sources = {
        {10, "Sensor1", "Radar"},
        {20, "Sensor2", "Radar"}
    };

    SelectDataSourcesWidget widget(mock, "Radars", "Radar");

    std::map<std::string, bool> selection;
    selection["10"] = true;
    selection["20"] = false;
    widget.updateSelected(selection);

    auto checkboxes = widget.findChildren<QCheckBox*>();
    REQUIRE(checkboxes.size() == 2);

    // Find checkbox by id property
    bool found_checked = false;
    bool found_unchecked = false;
    for (auto* cb : checkboxes)
    {
        unsigned int id = cb->property("id").toUInt();
        if (id == 10)
        {
            CHECK(cb->isChecked());
            found_checked = true;
        }
        else if (id == 20)
        {
            CHECK_FALSE(cb->isChecked());
            found_unchecked = true;
        }
    }
    CHECK(found_checked);
    CHECK(found_unchecked);
}

TEST_CASE("SelectDataSourcesWidget toggle emits selectionChangedSignal", "[gui][selectds]")
{
    MockDataSourceProvider mock;
    mock.sources = {
        {10, "Sensor1", "Radar"}
    };

    SelectDataSourcesWidget widget(mock, "Radars", "Radar");
    widget.updateSelected({});

    // Track signal emission via a connected slot
    bool signal_received = false;
    std::map<std::string, bool> received_selection;
    QObject::connect(&widget, &SelectDataSourcesWidget::selectionChangedSignal,
        [&](std::map<std::string, bool> sel) {
            signal_received = true;
            received_selection = sel;
        });

    // Find the checkbox and click it
    auto checkboxes = widget.findChildren<QCheckBox*>();
    REQUIRE(checkboxes.size() == 1);

    checkboxes[0]->click();
    CHECK(signal_received);
    CHECK(received_selection.count("10"));
    CHECK_FALSE(received_selection.at("10"));  // was checked, now unchecked
}

TEST_CASE("SelectDataSourcesWidget updateSelected clears old checkboxes", "[gui][selectds]")
{
    MockDataSourceProvider mock;
    mock.sources = {
        {1, "S1", "Radar"},
        {2, "S2", "Radar"},
        {3, "S3", "Radar"}
    };

    SelectDataSourcesWidget widget(mock, "Radars", "Radar");
    widget.updateSelected({});
    CHECK(widget.findChildren<QCheckBox*>().size() == 3);

    // Reduce to 1 source
    mock.sources = {{1, "S1", "Radar"}};
    widget.updateSelected({});
    CHECK(widget.findChildren<QCheckBox*>().size() == 1);
}

TEST_CASE("SelectDataSourcesWidget checkbox labels match source names", "[gui][selectds]")
{
    MockDataSourceProvider mock;
    mock.sources = {
        {1, "Vienna Radar", "Radar"},
        {2, "Graz MLAT",    "MLAT"}
    };

    SelectDataSourcesWidget widget(mock, "All", "Radar");
    widget.updateSelected({});

    auto checkboxes = widget.findChildren<QCheckBox*>();
    REQUIRE(checkboxes.size() == 1);
    CHECK(checkboxes[0]->text().toStdString() == "Vienna Radar");
}
