#include "catch.hpp"
#include "buffer.h"

// Number of values to add per NullableVector in multi-vector tests
static const unsigned int NUM_VALUES = 5;

// ============================================================================
// Buffer basics
// ============================================================================

TEST_CASE("Buffer empty construction", "[buffer]")
{
    PropertyList pl;
    Buffer buf(pl);
    REQUIRE(buf.size() == 0);
    REQUIRE(buf.properties().size() == 0);
}

TEST_CASE("Buffer construction with properties", "[buffer]")
{
    PropertyList pl;
    pl.addProperty("i", PropertyDataType::INT);
    pl.addProperty("d", PropertyDataType::DOUBLE);
    pl.addProperty("s", PropertyDataType::STRING);

    Buffer buf(pl);
    REQUIRE(buf.properties().size() == 3);
    REQUIRE(buf.has<int>("i"));
    REQUIRE(buf.has<double>("d"));
    REQUIRE(buf.has<std::string>("s"));
}

TEST_CASE("Buffer addProperty and hasProperty", "[buffer]")
{
    PropertyList pl;
    Buffer buf(pl);

    buf.addProperty("x", PropertyDataType::FLOAT);
    REQUIRE(buf.has<float>("x"));

    Property prop("y", PropertyDataType::UINT);
    REQUIRE_FALSE(buf.hasProperty(prop));
    buf.addProperty(prop);
    REQUIRE(buf.hasProperty(prop));
    REQUIRE(buf.has<unsigned int>("y"));
}

TEST_CASE("Buffer deleteProperty", "[buffer]")
{
    PropertyList pl;
    pl.addProperty("a", PropertyDataType::INT);
    pl.addProperty("b", PropertyDataType::DOUBLE);
    Buffer buf(pl);

    Property prop_a("a", PropertyDataType::INT);
    REQUIRE(buf.hasProperty(prop_a));

    buf.deleteProperty(prop_a);
    REQUIRE_FALSE(buf.hasProperty(prop_a));
    REQUIRE(buf.has<double>("b"));
}

TEST_CASE("Buffer size grows with data", "[buffer]")
{
    PropertyList pl;
    pl.addProperty("val", PropertyDataType::INT);
    Buffer buf(pl);

    REQUIRE(buf.size() == 0);
    buf.get<int>("val").set(0, 10);
    REQUIRE(buf.size() == 1);
    buf.get<int>("val").set(9, 20);
    REQUIRE(buf.size() == 10);
}

TEST_CASE("Buffer hasAnyPropertyNamed", "[buffer]")
{
    PropertyList pl;
    pl.addProperty("speed", PropertyDataType::DOUBLE);
    Buffer buf(pl);

    REQUIRE(buf.hasAnyPropertyNamed("speed"));
    REQUIRE_FALSE(buf.hasAnyPropertyNamed("altitude"));
}

TEST_CASE("Buffer rename", "[buffer]")
{
    PropertyList pl;
    pl.addProperty("old_name", PropertyDataType::INT);
    Buffer buf(pl);

    buf.get<int>("old_name").set(0, 42);
    buf.rename<int>("old_name", "new_name");

    REQUIRE(buf.has<int>("new_name"));
    REQUIRE_FALSE(buf.has<int>("old_name"));
    REQUIRE(buf.get<int>("new_name").get(0) == 42);
}

TEST_CASE("Buffer isNull", "[buffer]")
{
    PropertyList pl;
    pl.addProperty("val", PropertyDataType::INT);
    Buffer buf(pl);

    Property prop("val", PropertyDataType::INT);

    buf.get<int>("val").set(0, 10);
    // index 1 not set
    buf.get<int>("val").set(2, 30);

    REQUIRE_FALSE(buf.isNull(prop, 0));
    REQUIRE(buf.isNull(prop, 1));
    REQUIRE_FALSE(buf.isNull(prop, 2));
}

// ============================================================================
// Buffer with multiple NullableVectors per data type (5 each)
// Checks set indexes and unset indexes (must have isNull true)
// ============================================================================

TEST_CASE("Buffer multiple bool NullableVectors", "[buffer][multi][bool]")
{
    PropertyList pl;
    for (unsigned int i = 0; i < NUM_VALUES; ++i)
        pl.addProperty("bool_" + std::to_string(i), PropertyDataType::BOOL);
    Buffer buf(pl);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<bool>("bool_" + std::to_string(i));
        // set only even indexes (0, 2, 4, 6, 8)
        for (unsigned int idx = 0; idx < 10; idx += 2)
            nv.set(idx, (i % 2 == 0));
    }

    REQUIRE(buf.size() == 9); // max index 8 -> size 9

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<bool>("bool_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 9; ++idx)
        {
            if (idx % 2 == 0)
            {
                REQUIRE_FALSE(nv.isNull(idx));
                REQUIRE(nv.get(idx) == (i % 2 == 0));
            }
            else
            {
                REQUIRE(nv.isNull(idx));
            }
        }
    }
}

TEST_CASE("Buffer multiple char NullableVectors", "[buffer][multi][char]")
{
    PropertyList pl;
    for (unsigned int i = 0; i < NUM_VALUES; ++i)
        pl.addProperty("char_" + std::to_string(i), PropertyDataType::CHAR);
    Buffer buf(pl);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<char>("char_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 10; idx += 2)
            nv.set(idx, 'A' + static_cast<char>(i));
    }

    REQUIRE(buf.size() == 9);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<char>("char_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 9; ++idx)
        {
            if (idx % 2 == 0)
            {
                REQUIRE_FALSE(nv.isNull(idx));
                REQUIRE(nv.get(idx) == 'A' + static_cast<char>(i));
            }
            else
            {
                REQUIRE(nv.isNull(idx));
            }
        }
    }
}

TEST_CASE("Buffer multiple unsigned char NullableVectors", "[buffer][multi][uchar]")
{
    PropertyList pl;
    for (unsigned int i = 0; i < NUM_VALUES; ++i)
        pl.addProperty("uchar_" + std::to_string(i), PropertyDataType::UCHAR);
    Buffer buf(pl);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<unsigned char>("uchar_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 10; idx += 2)
            nv.set(idx, static_cast<unsigned char>(i * 50));
    }

    REQUIRE(buf.size() == 9);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<unsigned char>("uchar_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 9; ++idx)
        {
            if (idx % 2 == 0)
            {
                REQUIRE_FALSE(nv.isNull(idx));
                REQUIRE(nv.get(idx) == static_cast<unsigned char>(i * 50));
            }
            else
            {
                REQUIRE(nv.isNull(idx));
            }
        }
    }
}

TEST_CASE("Buffer multiple int NullableVectors", "[buffer][multi][int]")
{
    PropertyList pl;
    for (unsigned int i = 0; i < NUM_VALUES; ++i)
        pl.addProperty("int_" + std::to_string(i), PropertyDataType::INT);
    Buffer buf(pl);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<int>("int_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 10; idx += 2)
            nv.set(idx, static_cast<int>(i) * 100 + static_cast<int>(idx));
    }

    REQUIRE(buf.size() == 9);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<int>("int_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 9; ++idx)
        {
            if (idx % 2 == 0)
            {
                REQUIRE_FALSE(nv.isNull(idx));
                REQUIRE(nv.get(idx) == static_cast<int>(i) * 100 + static_cast<int>(idx));
            }
            else
            {
                REQUIRE(nv.isNull(idx));
            }
        }
    }
}

TEST_CASE("Buffer multiple unsigned int NullableVectors", "[buffer][multi][uint]")
{
    PropertyList pl;
    for (unsigned int i = 0; i < NUM_VALUES; ++i)
        pl.addProperty("uint_" + std::to_string(i), PropertyDataType::UINT);
    Buffer buf(pl);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<unsigned int>("uint_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 10; idx += 2)
            nv.set(idx, i * 1000u + idx);
    }

    REQUIRE(buf.size() == 9);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<unsigned int>("uint_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 9; ++idx)
        {
            if (idx % 2 == 0)
            {
                REQUIRE_FALSE(nv.isNull(idx));
                REQUIRE(nv.get(idx) == i * 1000u + idx);
            }
            else
            {
                REQUIRE(nv.isNull(idx));
            }
        }
    }
}

TEST_CASE("Buffer multiple long int NullableVectors", "[buffer][multi][longint]")
{
    PropertyList pl;
    for (unsigned int i = 0; i < NUM_VALUES; ++i)
        pl.addProperty("long_" + std::to_string(i), PropertyDataType::LONGINT);
    Buffer buf(pl);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<long int>("long_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 10; idx += 2)
            nv.set(idx, static_cast<long int>(i) * 100000L + static_cast<long int>(idx));
    }

    REQUIRE(buf.size() == 9);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<long int>("long_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 9; ++idx)
        {
            if (idx % 2 == 0)
            {
                REQUIRE_FALSE(nv.isNull(idx));
                REQUIRE(nv.get(idx) == static_cast<long int>(i) * 100000L + static_cast<long int>(idx));
            }
            else
            {
                REQUIRE(nv.isNull(idx));
            }
        }
    }
}

TEST_CASE("Buffer multiple unsigned long int NullableVectors", "[buffer][multi][ulongint]")
{
    PropertyList pl;
    for (unsigned int i = 0; i < NUM_VALUES; ++i)
        pl.addProperty("ulong_" + std::to_string(i), PropertyDataType::ULONGINT);
    Buffer buf(pl);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<unsigned long int>("ulong_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 10; idx += 2)
            nv.set(idx, static_cast<unsigned long int>(i) * 100000UL + idx);
    }

    REQUIRE(buf.size() == 9);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<unsigned long int>("ulong_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 9; ++idx)
        {
            if (idx % 2 == 0)
            {
                REQUIRE_FALSE(nv.isNull(idx));
                REQUIRE(nv.get(idx) == static_cast<unsigned long int>(i) * 100000UL + idx);
            }
            else
            {
                REQUIRE(nv.isNull(idx));
            }
        }
    }
}

TEST_CASE("Buffer multiple float NullableVectors", "[buffer][multi][float]")
{
    PropertyList pl;
    for (unsigned int i = 0; i < NUM_VALUES; ++i)
        pl.addProperty("float_" + std::to_string(i), PropertyDataType::FLOAT);
    Buffer buf(pl);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<float>("float_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 10; idx += 2)
            nv.set(idx, static_cast<float>(i) * 1.5f + static_cast<float>(idx) * 0.1f);
    }

    REQUIRE(buf.size() == 9);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<float>("float_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 9; ++idx)
        {
            if (idx % 2 == 0)
            {
                REQUIRE_FALSE(nv.isNull(idx));
                float expected = static_cast<float>(i) * 1.5f + static_cast<float>(idx) * 0.1f;
                REQUIRE(nv.get(idx) == Approx(expected));
            }
            else
            {
                REQUIRE(nv.isNull(idx));
            }
        }
    }
}

TEST_CASE("Buffer multiple double NullableVectors", "[buffer][multi][double]")
{
    PropertyList pl;
    for (unsigned int i = 0; i < NUM_VALUES; ++i)
        pl.addProperty("double_" + std::to_string(i), PropertyDataType::DOUBLE);
    Buffer buf(pl);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<double>("double_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 10; idx += 2)
            nv.set(idx, static_cast<double>(i) * 3.14 + static_cast<double>(idx) * 0.01);
    }

    REQUIRE(buf.size() == 9);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<double>("double_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 9; ++idx)
        {
            if (idx % 2 == 0)
            {
                REQUIRE_FALSE(nv.isNull(idx));
                double expected = static_cast<double>(i) * 3.14 + static_cast<double>(idx) * 0.01;
                REQUIRE(nv.get(idx) == Approx(expected));
            }
            else
            {
                REQUIRE(nv.isNull(idx));
            }
        }
    }
}

TEST_CASE("Buffer multiple string NullableVectors", "[buffer][multi][string]")
{
    PropertyList pl;
    for (unsigned int i = 0; i < NUM_VALUES; ++i)
        pl.addProperty("str_" + std::to_string(i), PropertyDataType::STRING);
    Buffer buf(pl);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<std::string>("str_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 10; idx += 2)
            nv.set(idx, "v" + std::to_string(i) + "_" + std::to_string(idx));
    }

    REQUIRE(buf.size() == 9);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<std::string>("str_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 9; ++idx)
        {
            if (idx % 2 == 0)
            {
                REQUIRE_FALSE(nv.isNull(idx));
                REQUIRE(nv.get(idx) == "v" + std::to_string(i) + "_" + std::to_string(idx));
            }
            else
            {
                REQUIRE(nv.isNull(idx));
            }
        }
    }
}

TEST_CASE("Buffer multiple json NullableVectors", "[buffer][multi][json]")
{
    PropertyList pl;
    for (unsigned int i = 0; i < NUM_VALUES; ++i)
        pl.addProperty("json_" + std::to_string(i), PropertyDataType::JSON);
    Buffer buf(pl);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<nlohmann::json>("json_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 10; idx += 2)
        {
            nlohmann::json j;
            j["vec"] = i;
            j["idx"] = idx;
            nv.set(idx, j);
        }
    }

    REQUIRE(buf.size() == 9);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<nlohmann::json>("json_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 9; ++idx)
        {
            if (idx % 2 == 0)
            {
                REQUIRE_FALSE(nv.isNull(idx));
                nlohmann::json expected;
                expected["vec"] = i;
                expected["idx"] = idx;
                REQUIRE(nv.get(idx) == expected);
            }
            else
            {
                REQUIRE(nv.isNull(idx));
            }
        }
    }
}

TEST_CASE("Buffer multiple ptime NullableVectors", "[buffer][multi][ptime]")
{
    PropertyList pl;
    for (unsigned int i = 0; i < NUM_VALUES; ++i)
        pl.addProperty("ts_" + std::to_string(i), PropertyDataType::TIMESTAMP);
    Buffer buf(pl);

    boost::gregorian::date base_date(2024, 1, 1);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<boost::posix_time::ptime>("ts_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 10; idx += 2)
        {
            boost::posix_time::ptime t(base_date,
                boost::posix_time::hours(i) + boost::posix_time::minutes(idx));
            nv.set(idx, t);
        }
    }

    REQUIRE(buf.size() == 9);

    for (unsigned int i = 0; i < NUM_VALUES; ++i)
    {
        auto& nv = buf.get<boost::posix_time::ptime>("ts_" + std::to_string(i));
        for (unsigned int idx = 0; idx < 9; ++idx)
        {
            if (idx % 2 == 0)
            {
                REQUIRE_FALSE(nv.isNull(idx));
                boost::posix_time::ptime expected(base_date,
                    boost::posix_time::hours(i) + boost::posix_time::minutes(idx));
                REQUIRE(nv.get(idx) == expected);
            }
            else
            {
                REQUIRE(nv.isNull(idx));
            }
        }
    }
}

// ============================================================================
// Buffer with all data types mixed together
// ============================================================================

TEST_CASE("Buffer all data types mixed", "[buffer][multi][alltypes]")
{
    PropertyList pl;
    pl.addProperty("b", PropertyDataType::BOOL);
    pl.addProperty("c", PropertyDataType::CHAR);
    pl.addProperty("uc", PropertyDataType::UCHAR);
    pl.addProperty("i", PropertyDataType::INT);
    pl.addProperty("ui", PropertyDataType::UINT);
    pl.addProperty("li", PropertyDataType::LONGINT);
    pl.addProperty("uli", PropertyDataType::ULONGINT);
    pl.addProperty("f", PropertyDataType::FLOAT);
    pl.addProperty("d", PropertyDataType::DOUBLE);
    pl.addProperty("s", PropertyDataType::STRING);
    pl.addProperty("j", PropertyDataType::JSON);
    pl.addProperty("ts", PropertyDataType::TIMESTAMP);

    Buffer buf(pl);
    REQUIRE(buf.properties().size() == 12);

    // set index 0 for all, leave index 1 null for all
    buf.get<bool>("b").set(0, true);
    buf.get<char>("c").set(0, 'X');
    buf.get<unsigned char>("uc").set(0, 128);
    buf.get<int>("i").set(0, -42);
    buf.get<unsigned int>("ui").set(0, 42u);
    buf.get<long int>("li").set(0, -123456L);
    buf.get<unsigned long int>("uli").set(0, 123456UL);
    buf.get<float>("f").set(0, 2.5f);
    buf.get<double>("d").set(0, 9.99);
    buf.get<std::string>("s").set(0, "mixed");
    buf.get<nlohmann::json>("j").set(0, nlohmann::json({{"k", "v"}}));

    boost::posix_time::ptime t(boost::gregorian::date(2024, 6, 15),
                                boost::posix_time::hours(8));
    buf.get<boost::posix_time::ptime>("ts").set(0, t);

    // force buffer to have at least 2 entries by setting null on index 1
    buf.get<int>("i").setNull(1);

    REQUIRE(buf.size() == 2);

    // verify set values
    REQUIRE(buf.get<bool>("b").get(0) == true);
    REQUIRE(buf.get<char>("c").get(0) == 'X');
    REQUIRE(buf.get<unsigned char>("uc").get(0) == 128);
    REQUIRE(buf.get<int>("i").get(0) == -42);
    REQUIRE(buf.get<unsigned int>("ui").get(0) == 42u);
    REQUIRE(buf.get<long int>("li").get(0) == -123456L);
    REQUIRE(buf.get<unsigned long int>("uli").get(0) == 123456UL);
    REQUIRE(buf.get<float>("f").get(0) == Approx(2.5f));
    REQUIRE(buf.get<double>("d").get(0) == Approx(9.99));
    REQUIRE(buf.get<std::string>("s").get(0) == "mixed");
    REQUIRE(buf.get<nlohmann::json>("j").get(0) == nlohmann::json({{"k", "v"}}));
    REQUIRE(buf.get<boost::posix_time::ptime>("ts").get(0) == t);

    // verify index 1 is null for all
    REQUIRE(buf.get<bool>("b").isNull(1));
    REQUIRE(buf.get<char>("c").isNull(1));
    REQUIRE(buf.get<unsigned char>("uc").isNull(1));
    REQUIRE(buf.get<int>("i").isNull(1));
    REQUIRE(buf.get<unsigned int>("ui").isNull(1));
    REQUIRE(buf.get<long int>("li").isNull(1));
    REQUIRE(buf.get<unsigned long int>("uli").isNull(1));
    REQUIRE(buf.get<float>("f").isNull(1));
    REQUIRE(buf.get<double>("d").isNull(1));
    REQUIRE(buf.get<std::string>("s").isNull(1));
    REQUIRE(buf.get<nlohmann::json>("j").isNull(1));
    REQUIRE(buf.get<boost::posix_time::ptime>("ts").isNull(1));
}

// ============================================================================
// Buffer cutToSize
// ============================================================================

TEST_CASE("Buffer cutToSize", "[buffer]")
{
    PropertyList pl;
    pl.addProperty("val", PropertyDataType::INT);
    pl.addProperty("name", PropertyDataType::STRING);
    Buffer buf(pl);

    for (unsigned int i = 0; i < 10; ++i)
    {
        buf.get<int>("val").set(i, static_cast<int>(i));
        buf.get<std::string>("name").set(i, "item_" + std::to_string(i));
    }

    REQUIRE(buf.size() == 10);

    buf.cutToSize(5);
    REQUIRE(buf.size() == 5);

    for (unsigned int i = 0; i < 5; ++i)
    {
        REQUIRE_FALSE(buf.get<int>("val").isNull(i));
        REQUIRE(buf.get<int>("val").get(i) == static_cast<int>(i));
    }
}

// ============================================================================
// Buffer seizeBuffer
// ============================================================================

TEST_CASE("Buffer seizeBuffer", "[buffer]")
{
    PropertyList pl;
    pl.addProperty("val", PropertyDataType::INT);
    Buffer buf1(pl);
    Buffer buf2(pl);

    for (unsigned int i = 0; i < 5; ++i)
        buf1.get<int>("val").set(i, static_cast<int>(i));

    for (unsigned int i = 0; i < 3; ++i)
        buf2.get<int>("val").set(i, static_cast<int>(i + 100));

    buf1.seizeBuffer(buf2);

    REQUIRE(buf1.size() == 8);

    // original data
    for (unsigned int i = 0; i < 5; ++i)
        REQUIRE(buf1.get<int>("val").get(i) == static_cast<int>(i));

    // seized data appended
    for (unsigned int i = 0; i < 3; ++i)
        REQUIRE(buf1.get<int>("val").get(5 + i) == static_cast<int>(i + 100));
}
