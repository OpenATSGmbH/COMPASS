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

#include <any>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <variant>
#include <map>

// Helper for if-constexpr test
template <typename T>
std::string type_name()
{
    if constexpr (std::is_integral_v<T>)
        return "integral";
    else if constexpr (std::is_floating_point_v<T>)
        return "floating";
    else
        return "other";
}

TEST_CASE("C++17 structured bindings", "[cpp17]")
{
    SECTION("tuple")
    {
        auto [a, b, c] = std::make_tuple(1, 2.5, std::string("hello"));
        REQUIRE(a == 1);
        REQUIRE(b == Approx(2.5));
        REQUIRE(c == "hello");
    }

    SECTION("pair")
    {
        std::map<std::string, int> m = {{"x", 10}};
        auto [it, inserted] = m.insert({"y", 20});
        REQUIRE(inserted);
        REQUIRE(it->second == 20);

        auto [it2, inserted2] = m.insert({"y", 30});
        REQUIRE_FALSE(inserted2);
        REQUIRE(it2->second == 20);
    }

    SECTION("struct")
    {
        struct Point { double x; double y; };
        auto [x, y] = Point{3.0, 4.0};
        REQUIRE(x == Approx(3.0));
        REQUIRE(y == Approx(4.0));
    }
}

TEST_CASE("C++17 std::optional", "[cpp17]")
{
    SECTION("engaged and disengaged")
    {
        std::optional<int> empty;
        std::optional<int> full = 42;

        REQUIRE_FALSE(empty.has_value());
        REQUIRE(full.has_value());
        REQUIRE(*full == 42);
        REQUIRE(full.value() == 42);
    }

    SECTION("value_or")
    {
        std::optional<std::string> empty;
        REQUIRE(empty.value_or("default") == "default");

        std::optional<std::string> full = "actual";
        REQUIRE(full.value_or("default") == "actual");
    }

    SECTION("emplace and reset")
    {
        std::optional<std::string> opt;
        opt.emplace("constructed");
        REQUIRE(opt.has_value());
        REQUIRE(*opt == "constructed");

        opt.reset();
        REQUIRE_FALSE(opt.has_value());
    }
}

TEST_CASE("C++17 std::variant", "[cpp17]")
{
    SECTION("holds_alternative and get")
    {
        std::variant<int, double, std::string> v = 42;
        REQUIRE(std::holds_alternative<int>(v));
        REQUIRE(std::get<int>(v) == 42);

        v = 3.14;
        REQUIRE(std::holds_alternative<double>(v));
        REQUIRE(std::get<double>(v) == Approx(3.14));

        v = std::string("hello");
        REQUIRE(std::holds_alternative<std::string>(v));
        REQUIRE(std::get<std::string>(v) == "hello");
    }

    SECTION("get_if")
    {
        std::variant<int, std::string> v = 10;
        REQUIRE(std::get_if<int>(&v) != nullptr);
        REQUIRE(*std::get_if<int>(&v) == 10);
        REQUIRE(std::get_if<std::string>(&v) == nullptr);
    }

    SECTION("visit")
    {
        std::variant<int, double> v = 5;
        auto result = std::visit([](auto val) -> double { return val * 2.0; }, v);
        REQUIRE(result == Approx(10.0));

        v = 1.5;
        result = std::visit([](auto val) -> double { return val * 2.0; }, v);
        REQUIRE(result == Approx(3.0));
    }
}

TEST_CASE("C++17 if constexpr", "[cpp17]")
{
    REQUIRE(type_name<int>() == "integral");
    REQUIRE(type_name<unsigned long>() == "integral");
    REQUIRE(type_name<double>() == "floating");
    REQUIRE(type_name<float>() == "floating");
    REQUIRE(type_name<std::string>() == "other");
}

TEST_CASE("C++17 std::string_view", "[cpp17]")
{
    SECTION("from string literal")
    {
        std::string_view sv = "hello world";
        REQUIRE(sv.size() == 11);
        REQUIRE(sv.substr(0, 5) == "hello");
        REQUIRE(sv.find("world") == 6);
    }

    SECTION("from std::string")
    {
        std::string s = "test string";
        std::string_view sv = s;
        REQUIRE(sv == "test string");
        REQUIRE(sv.data() == s.data());
    }

    SECTION("remove_prefix and remove_suffix")
    {
        std::string_view sv = "<<content>>";
        sv.remove_prefix(2);
        sv.remove_suffix(2);
        REQUIRE(sv == "content");
    }
}

TEST_CASE("C++17 std::any", "[cpp17]")
{
    SECTION("store and retrieve")
    {
        std::any a = 42;
        REQUIRE(a.has_value());
        REQUIRE(a.type() == typeid(int));
        REQUIRE(std::any_cast<int>(a) == 42);

        a = std::string("hello");
        REQUIRE(a.type() == typeid(std::string));
        REQUIRE(std::any_cast<std::string>(a) == "hello");
    }

    SECTION("bad cast throws")
    {
        std::any a = 42;
        REQUIRE_THROWS_AS(std::any_cast<std::string>(a), std::bad_any_cast);
    }

    SECTION("empty")
    {
        std::any a;
        REQUIRE_FALSE(a.has_value());

        a = 1;
        REQUIRE(a.has_value());

        a.reset();
        REQUIRE_FALSE(a.has_value());
    }
}

TEST_CASE("C++17 fold expressions", "[cpp17]")
{
    auto sum = [](auto... args) { return (args + ...); };
    REQUIRE(sum(1, 2, 3, 4) == 10);
    REQUIRE(sum(1.0, 2.5) == Approx(3.5));

    auto all = [](auto... args) { return (args && ...); };
    REQUIRE(all(true, true, true));
    REQUIRE_FALSE(all(true, false, true));
}

TEST_CASE("C++17 class template argument deduction", "[cpp17]")
{
    std::pair p{1, 2.0};
    REQUIRE(p.first == 1);
    REQUIRE(p.second == Approx(2.0));

    std::tuple t{1, 'a', 3.14};
    REQUIRE(std::get<0>(t) == 1);
    REQUIRE(std::get<1>(t) == 'a');
    REQUIRE(std::get<2>(t) == Approx(3.14));
}
