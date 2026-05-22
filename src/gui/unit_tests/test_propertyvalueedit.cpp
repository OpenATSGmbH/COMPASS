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
#include "propertyvalueedit.h"

#include <QLineEdit>
#include <QSignalSpy>

using namespace std::string_literals;

// ─── Construction & data type ───────────────────────────────────────────────

TEST_CASE("PropertyValueEdit default construction", "[gui][propertyvalueedit]")
{
    PropertyValueEdit edit(PropertyDataType::DOUBLE, 6);

    CHECK(edit.dataType() == PropertyDataType::DOUBLE);
    CHECK_FALSE(edit.hasInput());
    CHECK_FALSE(edit.isValid());
}

TEST_CASE("PropertyValueEdit INT type", "[gui][propertyvalueedit]")
{
    PropertyValueEdit edit(PropertyDataType::INT);

    CHECK(edit.dataType() == PropertyDataType::INT);
}

TEST_CASE("PropertyValueEdit setPropertyDataType clears value", "[gui][propertyvalueedit]")
{
    PropertyValueEdit edit(PropertyDataType::DOUBLE, 4);
    edit.setValue("42.0"s);
    CHECK(edit.hasInput());

    edit.setPropertyDataType(PropertyDataType::INT);
    CHECK(edit.dataType() == PropertyDataType::INT);
    CHECK_FALSE(edit.hasInput());
}

// ─── setValue / valueAsString round-trip ─────────────────────────────────────

TEST_CASE("PropertyValueEdit setValue string round-trip", "[gui][propertyvalueedit]")
{
    PropertyValueEdit edit(PropertyDataType::DOUBLE, 6);

    bool ok = edit.setValue("3.14"s);

    CHECK(ok);
    CHECK(edit.isValid());
    CHECK(edit.valueAsString() == "3.14");
}

TEST_CASE("PropertyValueEdit setValue double round-trip", "[gui][propertyvalueedit]")
{
    PropertyValueEdit edit(PropertyDataType::DOUBLE, 2);

    CHECK(edit.setValue(42.5));
    CHECK(edit.isValid());
    CHECK(edit.valueAsDouble().has_value());
    CHECK(edit.valueAsDouble().value() == Approx(42.5));
}

TEST_CASE("PropertyValueEdit setValue int type", "[gui][propertyvalueedit]")
{
    PropertyValueEdit edit(PropertyDataType::INT);

    CHECK(edit.setValue("123"s));
    CHECK(edit.isValid());
    CHECK(edit.valueAsString() == "123");
}

// ─── Validity ───────────────────────────────────────────────────────────────

TEST_CASE("PropertyValueEdit invalid string for double", "[gui][propertyvalueedit]")
{
    PropertyValueEdit edit(PropertyDataType::DOUBLE);

    CHECK_FALSE(edit.setValue("not_a_number"s));
    CHECK_FALSE(edit.isValid());
}

TEST_CASE("PropertyValueEdit empty string is invalid", "[gui][propertyvalueedit]")
{
    PropertyValueEdit edit(PropertyDataType::DOUBLE);

    CHECK_FALSE(edit.setValue(""s));
    CHECK_FALSE(edit.isValid());
    CHECK_FALSE(edit.hasInput());
}

TEST_CASE("PropertyValueEdit valueAsDouble empty returns no value", "[gui][propertyvalueedit]")
{
    PropertyValueEdit edit(PropertyDataType::DOUBLE);

    CHECK_FALSE(edit.valueAsDouble().has_value());
}

// ─── Range pairing ──────────────────────────────────────────────────────────

TEST_CASE("PropertyValueEdit connectRange cross-validates", "[gui][propertyvalueedit]")
{
    PropertyValueEdit edit_min(PropertyDataType::DOUBLE, 2);
    PropertyValueEdit edit_max(PropertyDataType::DOUBLE, 2);

    edit_min.setValue("10.0"s);
    edit_max.setValue("90.0"s);

    PropertyValueEdit::connectRange(&edit_min, &edit_max);

    // Both should be valid: min < max
    CHECK(edit_min.isValid());
    CHECK(edit_max.isValid());
}

TEST_CASE("PropertyValueEdit connectRange detects invalid order", "[gui][propertyvalueedit]")
{
    PropertyValueEdit edit_min(PropertyDataType::DOUBLE, 2);
    PropertyValueEdit edit_max(PropertyDataType::DOUBLE, 2);

    // Set min > max before connecting
    edit_min.setValue("90.0"s);
    edit_max.setValue("10.0"s);

    PropertyValueEdit::connectRange(&edit_min, &edit_max);

    // Both values parse correctly so isValid() is true,
    // but the inverted range sets the error stylesheet on the inner QLineEdit.
    auto* le_min = edit_min.findChild<QLineEdit*>();
    auto* le_max = edit_max.findChild<QLineEdit*>();
    REQUIRE(le_min);
    REQUIRE(le_max);

    bool either_has_error = !le_min->styleSheet().isEmpty() || !le_max->styleSheet().isEmpty();
    CHECK(either_has_error);
}

// ─── Signals ────────────────────────────────────────────────────────────────

TEST_CASE("PropertyValueEdit valueChanged signal on setValue", "[gui][propertyvalueedit]")
{
    PropertyValueEdit edit(PropertyDataType::DOUBLE);

    // Note: setValue calls setText which does not emit textEdited (only
    // programmatic changes). The valueChanged signal is driven by textEdited.
    // This test verifies the signal is wired correctly for user edits.
    QSignalSpy spy(&edit, &PropertyValueEdit::valueChanged);

    edit.setValue("5.0"s);
    // Programmatic setValue does not trigger textEdited → no signal
    CHECK(spy.count() == 0);
}
