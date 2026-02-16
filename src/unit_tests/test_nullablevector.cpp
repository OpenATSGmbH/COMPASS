#include "catch.hpp"
#include "buffer.h"

// Helper: create a Buffer with a single property of the given type and name
static Buffer makeBuffer(const std::string& name, PropertyDataType type)
{
    PropertyList pl;
    pl.addProperty(name, type);
    return Buffer(pl);
}

// ============================================================================
// NullableVector<bool>
// ============================================================================

TEST_CASE("NullableVector<bool> set and get", "[nullablevector][bool]")
{
    Buffer buf = makeBuffer("flag", PropertyDataType::BOOL);
    auto& nv = buf.get<bool>("flag");

    REQUIRE(buf.size() == 0);

    nv.set(0, true);
    REQUIRE(buf.size() == 1);
    REQUIRE_FALSE(nv.isNull(0));
    REQUIRE(nv.get(0) == true);

    nv.set(1, false);
    REQUIRE(buf.size() == 2);
    REQUIRE(nv.get(1) == false);
}

TEST_CASE("NullableVector<bool> null semantics", "[nullablevector][bool]")
{
    Buffer buf = makeBuffer("flag", PropertyDataType::BOOL);
    auto& nv = buf.get<bool>("flag");

    // unset index is null
    REQUIRE(nv.isNull(0));

    nv.set(0, true);
    REQUIRE_FALSE(nv.isNull(0));

    nv.setNull(0);
    REQUIRE(nv.isNull(0));
}

TEST_CASE("NullableVector<bool> sparse set leaves gaps null", "[nullablevector][bool]")
{
    Buffer buf = makeBuffer("flag", PropertyDataType::BOOL);
    auto& nv = buf.get<bool>("flag");

    nv.set(3, true);
    REQUIRE(buf.size() == 4);
    REQUIRE(nv.isNull(0));
    REQUIRE(nv.isNull(1));
    REQUIRE(nv.isNull(2));
    REQUIRE_FALSE(nv.isNull(3));
    REQUIRE(nv.get(3) == true);
}

TEST_CASE("NullableVector<bool> isAlwaysNull / isNeverNull", "[nullablevector][bool]")
{
    Buffer buf = makeBuffer("flag", PropertyDataType::BOOL);
    auto& nv = buf.get<bool>("flag");

    REQUIRE(nv.isAlwaysNull());

    nv.set(0, true);
    nv.set(1, false);
    REQUIRE(nv.isNeverNull());
    REQUIRE_FALSE(nv.isAlwaysNull());

    nv.setNull(0);
    REQUIRE_FALSE(nv.isNeverNull());
    REQUIRE_FALSE(nv.isAlwaysNull());
}

// ============================================================================
// NullableVector<char>
// ============================================================================

TEST_CASE("NullableVector<char> set and get", "[nullablevector][char]")
{
    Buffer buf = makeBuffer("ch", PropertyDataType::CHAR);
    auto& nv = buf.get<char>("ch");

    nv.set(0, 'A');
    nv.set(1, 'Z');
    REQUIRE(nv.get(0) == 'A');
    REQUIRE(nv.get(1) == 'Z');
    REQUIRE_FALSE(nv.isNull(0));
    REQUIRE_FALSE(nv.isNull(1));
}

TEST_CASE("NullableVector<char> null semantics", "[nullablevector][char]")
{
    Buffer buf = makeBuffer("ch", PropertyDataType::CHAR);
    auto& nv = buf.get<char>("ch");

    nv.set(2, 'X');
    REQUIRE(nv.isNull(0));
    REQUIRE(nv.isNull(1));
    REQUIRE_FALSE(nv.isNull(2));

    nv.setNull(2);
    REQUIRE(nv.isNull(2));
}

// ============================================================================
// NullableVector<unsigned char>
// ============================================================================

TEST_CASE("NullableVector<unsigned char> set and get", "[nullablevector][uchar]")
{
    Buffer buf = makeBuffer("uch", PropertyDataType::UCHAR);
    auto& nv = buf.get<unsigned char>("uch");

    nv.set(0, 0);
    nv.set(1, 255);
    REQUIRE(nv.get(0) == 0);
    REQUIRE(nv.get(1) == 255);
}

TEST_CASE("NullableVector<unsigned char> null semantics", "[nullablevector][uchar]")
{
    Buffer buf = makeBuffer("uch", PropertyDataType::UCHAR);
    auto& nv = buf.get<unsigned char>("uch");

    nv.set(3, 42);
    REQUIRE(nv.isNull(0));
    REQUIRE(nv.isNull(1));
    REQUIRE(nv.isNull(2));
    REQUIRE_FALSE(nv.isNull(3));
}

// ============================================================================
// NullableVector<int>
// ============================================================================

TEST_CASE("NullableVector<int> set and get", "[nullablevector][int]")
{
    Buffer buf = makeBuffer("val", PropertyDataType::INT);
    auto& nv = buf.get<int>("val");

    nv.set(0, -100);
    nv.set(1, 0);
    nv.set(2, 100);
    REQUIRE(nv.get(0) == -100);
    REQUIRE(nv.get(1) == 0);
    REQUIRE(nv.get(2) == 100);
}

TEST_CASE("NullableVector<int> null semantics", "[nullablevector][int]")
{
    Buffer buf = makeBuffer("val", PropertyDataType::INT);
    auto& nv = buf.get<int>("val");

    nv.set(5, 42);
    for (unsigned int i = 0; i < 5; ++i)
        REQUIRE(nv.isNull(i));

    REQUIRE_FALSE(nv.isNull(5));
    REQUIRE(nv.get(5) == 42);
}

TEST_CASE("NullableVector<int> overwrite value", "[nullablevector][int]")
{
    Buffer buf = makeBuffer("val", PropertyDataType::INT);
    auto& nv = buf.get<int>("val");

    nv.set(0, 10);
    REQUIRE(nv.get(0) == 10);
    nv.set(0, 20);
    REQUIRE(nv.get(0) == 20);
}

TEST_CASE("NullableVector<int> setAll and setAllNull", "[nullablevector][int]")
{
    Buffer buf = makeBuffer("val", PropertyDataType::INT);
    auto& nv = buf.get<int>("val");

    nv.set(0, 1);
    nv.set(1, 2);
    nv.set(2, 3);

    nv.setAll(99);
    REQUIRE(nv.get(0) == 99);
    REQUIRE(nv.get(1) == 99);
    REQUIRE(nv.get(2) == 99);
    REQUIRE(nv.isNeverNull());

    nv.setAllNull();
    REQUIRE(nv.isNull(0));
    REQUIRE(nv.isNull(1));
    REQUIRE(nv.isNull(2));
    REQUIRE(nv.isAlwaysNull());
}

TEST_CASE("NullableVector<int> getRef", "[nullablevector][int]")
{
    Buffer buf = makeBuffer("val", PropertyDataType::INT);
    auto& nv = buf.get<int>("val");

    nv.set(0, 5);
    int& ref = nv.getRef(0);
    REQUIRE(ref == 5);
    ref = 10;
    REQUIRE(nv.get(0) == 10);
}

// ============================================================================
// NullableVector<unsigned int>
// ============================================================================

TEST_CASE("NullableVector<unsigned int> set and get", "[nullablevector][uint]")
{
    Buffer buf = makeBuffer("uval", PropertyDataType::UINT);
    auto& nv = buf.get<unsigned int>("uval");

    nv.set(0, 0u);
    nv.set(1, 4294967295u);
    REQUIRE(nv.get(0) == 0u);
    REQUIRE(nv.get(1) == 4294967295u);
}

TEST_CASE("NullableVector<unsigned int> null semantics", "[nullablevector][uint]")
{
    Buffer buf = makeBuffer("uval", PropertyDataType::UINT);
    auto& nv = buf.get<unsigned int>("uval");

    nv.set(2, 100u);
    REQUIRE(nv.isNull(0));
    REQUIRE(nv.isNull(1));
    REQUIRE_FALSE(nv.isNull(2));
}

// ============================================================================
// NullableVector<long int>
// ============================================================================

TEST_CASE("NullableVector<long int> set and get", "[nullablevector][longint]")
{
    Buffer buf = makeBuffer("lval", PropertyDataType::LONGINT);
    auto& nv = buf.get<long int>("lval");

    nv.set(0, -999999999L);
    nv.set(1, 999999999L);
    REQUIRE(nv.get(0) == -999999999L);
    REQUIRE(nv.get(1) == 999999999L);
}

TEST_CASE("NullableVector<long int> null semantics", "[nullablevector][longint]")
{
    Buffer buf = makeBuffer("lval", PropertyDataType::LONGINT);
    auto& nv = buf.get<long int>("lval");

    nv.set(4, 42L);
    for (unsigned int i = 0; i < 4; ++i)
        REQUIRE(nv.isNull(i));
    REQUIRE_FALSE(nv.isNull(4));
}

// ============================================================================
// NullableVector<unsigned long int>
// ============================================================================

TEST_CASE("NullableVector<unsigned long int> set and get", "[nullablevector][ulongint]")
{
    Buffer buf = makeBuffer("ulval", PropertyDataType::ULONGINT);
    auto& nv = buf.get<unsigned long int>("ulval");

    nv.set(0, 0UL);
    nv.set(1, 18446744073709551615UL);
    REQUIRE(nv.get(0) == 0UL);
    REQUIRE(nv.get(1) == 18446744073709551615UL);
}

TEST_CASE("NullableVector<unsigned long int> null semantics", "[nullablevector][ulongint]")
{
    Buffer buf = makeBuffer("ulval", PropertyDataType::ULONGINT);
    auto& nv = buf.get<unsigned long int>("ulval");

    nv.set(3, 123UL);
    REQUIRE(nv.isNull(0));
    REQUIRE(nv.isNull(1));
    REQUIRE(nv.isNull(2));
    REQUIRE_FALSE(nv.isNull(3));
}

// ============================================================================
// NullableVector<float>
// ============================================================================

TEST_CASE("NullableVector<float> set and get", "[nullablevector][float]")
{
    Buffer buf = makeBuffer("fval", PropertyDataType::FLOAT);
    auto& nv = buf.get<float>("fval");

    nv.set(0, 3.14f);
    nv.set(1, -2.5f);
    nv.set(2, 0.0f);
    REQUIRE(nv.get(0) == Approx(3.14f));
    REQUIRE(nv.get(1) == Approx(-2.5f));
    REQUIRE(nv.get(2) == Approx(0.0f));
}

TEST_CASE("NullableVector<float> null semantics", "[nullablevector][float]")
{
    Buffer buf = makeBuffer("fval", PropertyDataType::FLOAT);
    auto& nv = buf.get<float>("fval");

    nv.set(2, 1.0f);
    REQUIRE(nv.isNull(0));
    REQUIRE(nv.isNull(1));
    REQUIRE_FALSE(nv.isNull(2));

    nv.setNull(2);
    REQUIRE(nv.isNull(2));
    REQUIRE(nv.isAlwaysNull());
}

// ============================================================================
// NullableVector<double>
// ============================================================================

TEST_CASE("NullableVector<double> set and get", "[nullablevector][double]")
{
    Buffer buf = makeBuffer("dval", PropertyDataType::DOUBLE);
    auto& nv = buf.get<double>("dval");

    nv.set(0, 1.23456789012345);
    nv.set(1, -9.87654321);
    REQUIRE(nv.get(0) == Approx(1.23456789012345));
    REQUIRE(nv.get(1) == Approx(-9.87654321));
}

TEST_CASE("NullableVector<double> null semantics", "[nullablevector][double]")
{
    Buffer buf = makeBuffer("dval", PropertyDataType::DOUBLE);
    auto& nv = buf.get<double>("dval");

    nv.set(4, 99.9);
    for (unsigned int i = 0; i < 4; ++i)
        REQUIRE(nv.isNull(i));
    REQUIRE_FALSE(nv.isNull(4));
}

TEST_CASE("NullableVector<double> operator*=", "[nullablevector][double]")
{
    Buffer buf = makeBuffer("dval", PropertyDataType::DOUBLE);
    auto& nv = buf.get<double>("dval");

    nv.set(0, 10.0);
    nv.set(1, 20.0);
    // leave index 2 null by setting index 3
    nv.set(3, 30.0);

    nv *= 2.0;
    REQUIRE(nv.get(0) == Approx(20.0));
    REQUIRE(nv.get(1) == Approx(40.0));
    REQUIRE(nv.isNull(2));
    REQUIRE(nv.get(3) == Approx(60.0));
}

// ============================================================================
// NullableVector<std::string>
// ============================================================================

TEST_CASE("NullableVector<string> set and get", "[nullablevector][string]")
{
    Buffer buf = makeBuffer("sval", PropertyDataType::STRING);
    auto& nv = buf.get<std::string>("sval");

    nv.set(0, "hello");
    nv.set(1, "world");
    nv.set(2, "");
    REQUIRE(nv.get(0) == "hello");
    REQUIRE(nv.get(1) == "world");
    REQUIRE(nv.get(2) == "");
}

TEST_CASE("NullableVector<string> null semantics", "[nullablevector][string]")
{
    Buffer buf = makeBuffer("sval", PropertyDataType::STRING);
    auto& nv = buf.get<std::string>("sval");

    nv.set(3, "test");
    REQUIRE(nv.isNull(0));
    REQUIRE(nv.isNull(1));
    REQUIRE(nv.isNull(2));
    REQUIRE_FALSE(nv.isNull(3));
    REQUIRE(nv.get(3) == "test");

    nv.setNull(3);
    REQUIRE(nv.isNull(3));
}

TEST_CASE("NullableVector<string> getRef", "[nullablevector][string]")
{
    Buffer buf = makeBuffer("sval", PropertyDataType::STRING);
    auto& nv = buf.get<std::string>("sval");

    nv.set(0, "original");
    std::string& ref = nv.getRef(0);
    REQUIRE(ref == "original");
    ref = "modified";
    REQUIRE(nv.get(0) == "modified");
}

// ============================================================================
// NullableVector<nlohmann::json>
// ============================================================================

TEST_CASE("NullableVector<json> set and get", "[nullablevector][json]")
{
    Buffer buf = makeBuffer("jval", PropertyDataType::JSON);
    auto& nv = buf.get<nlohmann::json>("jval");

    nlohmann::json j1 = {{"key", "value"}, {"num", 42}};
    nlohmann::json j2 = nlohmann::json::array({1, 2, 3});

    nv.set(0, j1);
    nv.set(1, j2);
    REQUIRE(nv.get(0) == j1);
    REQUIRE(nv.get(1) == j2);
}

TEST_CASE("NullableVector<json> null semantics", "[nullablevector][json]")
{
    Buffer buf = makeBuffer("jval", PropertyDataType::JSON);
    auto& nv = buf.get<nlohmann::json>("jval");

    nlohmann::json j = {{"a", 1}};
    nv.set(2, j);
    REQUIRE(nv.isNull(0));
    REQUIRE(nv.isNull(1));
    REQUIRE_FALSE(nv.isNull(2));
}

// ============================================================================
// NullableVector<boost::posix_time::ptime>
// ============================================================================

TEST_CASE("NullableVector<ptime> set and get", "[nullablevector][ptime]")
{
    Buffer buf = makeBuffer("ts", PropertyDataType::TIMESTAMP);
    auto& nv = buf.get<boost::posix_time::ptime>("ts");

    boost::posix_time::ptime t1(boost::gregorian::date(2024, 1, 15),
                                 boost::posix_time::hours(10) + boost::posix_time::minutes(30));
    boost::posix_time::ptime t2(boost::gregorian::date(2024, 6, 30),
                                 boost::posix_time::hours(23) + boost::posix_time::minutes(59));

    nv.set(0, t1);
    nv.set(1, t2);
    REQUIRE(nv.get(0) == t1);
    REQUIRE(nv.get(1) == t2);
}

TEST_CASE("NullableVector<ptime> null semantics", "[nullablevector][ptime]")
{
    Buffer buf = makeBuffer("ts", PropertyDataType::TIMESTAMP);
    auto& nv = buf.get<boost::posix_time::ptime>("ts");

    boost::posix_time::ptime t(boost::gregorian::date(2024, 3, 1),
                                boost::posix_time::hours(12));
    nv.set(2, t);
    REQUIRE(nv.isNull(0));
    REQUIRE(nv.isNull(1));
    REQUIRE_FALSE(nv.isNull(2));
    REQUIRE(nv.get(2) == t);
}

// ============================================================================
// NullableVector distinctValues / contentSize / clearData
// ============================================================================

TEST_CASE("NullableVector<int> distinctValues", "[nullablevector][int]")
{
    Buffer buf = makeBuffer("val", PropertyDataType::INT);
    auto& nv = buf.get<int>("val");

    nv.set(0, 10);
    nv.set(1, 20);
    nv.set(2, 10);
    nv.set(3, 30);
    // index 4 null
    nv.set(5, 20);

    auto dv = nv.distinctValues();
    REQUIRE(dv.size() == 3);
    REQUIRE(dv.count(10) == 1);
    REQUIRE(dv.count(20) == 1);
    REQUIRE(dv.count(30) == 1);
}

TEST_CASE("NullableVector<int> distinctValuesWithCounts", "[nullablevector][int]")
{
    Buffer buf = makeBuffer("val", PropertyDataType::INT);
    auto& nv = buf.get<int>("val");

    nv.set(0, 10);
    nv.set(1, 20);
    nv.set(2, 10);
    nv.set(3, 30);

    auto dvc = nv.distinctValuesWithCounts();
    REQUIRE(dvc[10] == 2);
    REQUIRE(dvc[20] == 1);
    REQUIRE(dvc[30] == 1);
}

TEST_CASE("NullableVector<double> minMaxValues", "[nullablevector][double]")
{
    Buffer buf = makeBuffer("dval", PropertyDataType::DOUBLE);
    auto& nv = buf.get<double>("dval");

    nv.set(0, 5.0);
    nv.set(1, -3.0);
    // index 2 null
    nv.set(3, 10.0);

    auto [valid, min_val, max_val] = nv.minMaxValues();
    REQUIRE(valid);
    REQUIRE(min_val == Approx(-3.0));
    REQUIRE(max_val == Approx(10.0));
}

TEST_CASE("NullableVector<int> contentSize", "[nullablevector][int]")
{
    Buffer buf = makeBuffer("val", PropertyDataType::INT);
    auto& nv = buf.get<int>("val");

    REQUIRE(nv.contentSize() == 0);
    nv.set(0, 1);
    REQUIRE(nv.contentSize() == 1);
    nv.set(5, 2);
    REQUIRE(nv.contentSize() == 6);
}

TEST_CASE("NullableVector<int> clearData", "[nullablevector][int]")
{
    Buffer buf = makeBuffer("val", PropertyDataType::INT);
    auto& nv = buf.get<int>("val");

    nv.set(0, 1);
    nv.set(1, 2);
    REQUIRE(nv.contentSize() == 2);

    nv.clearData();
    REQUIRE(nv.contentSize() == 0);
    REQUIRE(nv.isNull(0));
    REQUIRE(nv.isNull(1));
}
