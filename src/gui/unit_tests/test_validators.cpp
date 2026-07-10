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
#include "traced_assert.h"
#include "textfielddoublevalidator.h"
#include "textfieldhexvalidator.h"
#include "textfieldoctvalidator.h"

// Helper to call validate without caring about pos
static QValidator::State validate(const QValidator& v, const QString& input)
{
    QString s = input;
    int pos = 0;
    return v.validate(s, pos);
}

// ─── TextFieldDoubleValidator ───────────────────────────────────────────────

TEST_CASE("TextFieldDoubleValidator accepts value in range", "[gui][validator][double]")
{
    TextFieldDoubleValidator v(0.0, 100.0, 2);

    CHECK(validate(v, "50.00") == QValidator::Acceptable);
    CHECK(validate(v, "0.00")  == QValidator::Acceptable);
    CHECK(validate(v, "100.00") == QValidator::Acceptable);
}

TEST_CASE("TextFieldDoubleValidator rejects value out of range", "[gui][validator][double]")
{
    TextFieldDoubleValidator v(0.0, 100.0, 2);

    CHECK(validate(v, "100.01") == QValidator::Invalid);
    CHECK(validate(v, "-1.00")  == QValidator::Invalid);
    CHECK(validate(v, "200.00") == QValidator::Invalid);
}

TEST_CASE("TextFieldDoubleValidator limits decimal places", "[gui][validator][double]")
{
    TextFieldDoubleValidator v(0.0, 100.0, 2);

    CHECK(validate(v, "3.14")  == QValidator::Acceptable);
    CHECK(validate(v, "3.141") == QValidator::Invalid);
}

TEST_CASE("TextFieldDoubleValidator treats empty and minus-only as intermediate", "[gui][validator][double]")
{
    TextFieldDoubleValidator v(-100.0, 100.0, 2);

    CHECK(validate(v, "")  == QValidator::Intermediate);
    CHECK(validate(v, "-") == QValidator::Intermediate);
}

TEST_CASE("TextFieldDoubleValidator leading zero handling", "[gui][validator][double]")
{
    TextFieldDoubleValidator v(0.0, 100.0, 2);

    CHECK(validate(v, "0")  == QValidator::Acceptable);
    CHECK(validate(v, "0.") == QValidator::Acceptable);
}

TEST_CASE("TextFieldDoubleValidator integer mode (decimals=0)", "[gui][validator][double]")
{
    TextFieldDoubleValidator v(0.0, 10.0, 0);

    CHECK(validate(v, "5")  == QValidator::Acceptable);
    CHECK(validate(v, "10") == QValidator::Acceptable);
    CHECK(validate(v, "11") == QValidator::Invalid);
}

TEST_CASE("TextFieldDoubleValidator negative range", "[gui][validator][double]")
{
    TextFieldDoubleValidator v(-50.0, -10.0, 1);

    CHECK(validate(v, "-30.0") == QValidator::Acceptable);
    CHECK(validate(v, "-5.0")  == QValidator::Invalid);
    CHECK(validate(v, "0.0")   == QValidator::Invalid);
}

// ─── TextFieldHexValidator ──────────────────────────────────────────────────

TEST_CASE("TextFieldHexValidator accepts valid hex", "[gui][validator][hex]")
{
    TextFieldHexValidator v(6);

    CHECK(validate(v, "3c6752") == QValidator::Acceptable);
    CHECK(validate(v, "ABCDEF") == QValidator::Acceptable);
    CHECK(validate(v, "0")      == QValidator::Acceptable);
    CHECK(validate(v, "aF09")   == QValidator::Acceptable);
}

TEST_CASE("TextFieldHexValidator rejects invalid characters", "[gui][validator][hex]")
{
    TextFieldHexValidator v(6);

    CHECK(validate(v, "GHIJKL") == QValidator::Invalid);
    CHECK(validate(v, "12345g") == QValidator::Invalid);
    CHECK(validate(v, "zzzz")   == QValidator::Invalid);
}

TEST_CASE("TextFieldHexValidator enforces max length", "[gui][validator][hex]")
{
    TextFieldHexValidator v(6);

    CHECK(validate(v, "123456")  == QValidator::Acceptable);
    CHECK(validate(v, "1234567") == QValidator::Invalid);
}

TEST_CASE("TextFieldHexValidator treats empty string as intermediate", "[gui][validator][hex]")
{
    TextFieldHexValidator v(6);

    CHECK(validate(v, "") == QValidator::Intermediate);
}

// ─── TextFieldOctValidator ──────────────────────────────────────────────────

TEST_CASE("TextFieldOctValidator accepts valid octal", "[gui][validator][oct]")
{
    TextFieldOctValidator v(4);

    CHECK(validate(v, "7000") == QValidator::Acceptable);
    CHECK(validate(v, "0123") == QValidator::Acceptable);
    CHECK(validate(v, "7")    == QValidator::Acceptable);
}

TEST_CASE("TextFieldOctValidator rejects non-octal digits", "[gui][validator][oct]")
{
    TextFieldOctValidator v(4);

    CHECK(validate(v, "8000") == QValidator::Invalid);
    CHECK(validate(v, "9")    == QValidator::Invalid);
    CHECK(validate(v, "abcd") == QValidator::Invalid);
}

TEST_CASE("TextFieldOctValidator enforces max length", "[gui][validator][oct]")
{
    TextFieldOctValidator v(4);

    CHECK(validate(v, "7777")  == QValidator::Acceptable);
    CHECK(validate(v, "77777") == QValidator::Invalid);
}

TEST_CASE("TextFieldOctValidator treats empty string as intermediate", "[gui][validator][oct]")
{
    TextFieldOctValidator v(4);

    CHECK(validate(v, "") == QValidator::Intermediate);
}
