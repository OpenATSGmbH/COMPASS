#include "catch.hpp"
#include "buffer.h"

static Buffer makeStrBuffer(const std::string& name = "sv")
{
    PropertyList pl;
    pl.addProperty(name, PropertyDataType::STRING);
    return Buffer(pl);
}

// ============================================================================
// Basic set / get
// ============================================================================

TEST_CASE("DictNullableVector set and get", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    dv.set(0, "hello");
    dv.set(1, "world");
    dv.set(2, "");
    REQUIRE(dv.get(0) == "hello");
    REQUIRE(dv.get(1) == "world");
    REQUIRE(dv.get(2) == "");
    REQUIRE(buf.size() == 3);
}

TEST_CASE("DictNullableVector duplicate values share dictionary entry", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    dv.set(0, "AAA");
    dv.set(1, "BBB");
    dv.set(2, "AAA");
    dv.set(3, "CCC");
    dv.set(4, "BBB");

    REQUIRE(dv.dictionarySize() == 3);
    REQUIRE(dv.get(0) == "AAA");
    REQUIRE(dv.get(2) == "AAA");
    REQUIRE(dv.get(1) == "BBB");
    REQUIRE(dv.get(4) == "BBB");
    REQUIRE(dv.get(3) == "CCC");
}

TEST_CASE("DictNullableVector overwrite value", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    dv.set(0, "first");
    REQUIRE(dv.get(0) == "first");

    dv.set(0, "second");
    REQUIRE(dv.get(0) == "second");

    // both values remain in dictionary (no GC)
    REQUIRE(dv.dictionarySize() == 2);
}

// ============================================================================
// Null semantics
// ============================================================================

TEST_CASE("DictNullableVector null semantics", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    // unset index is null
    REQUIRE(dv.isNull(0));

    dv.set(3, "test");
    REQUIRE(dv.isNull(0));
    REQUIRE(dv.isNull(1));
    REQUIRE(dv.isNull(2));
    REQUIRE_FALSE(dv.isNull(3));
    REQUIRE(dv.get(3) == "test");

    dv.setNull(3);
    REQUIRE(dv.isNull(3));
}

TEST_CASE("DictNullableVector isAlwaysNull / isNeverNull", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    REQUIRE(dv.isAlwaysNull());
    REQUIRE(dv.isNeverNull()); // size 0 -> trivially true

    dv.set(0, "a");
    REQUIRE_FALSE(dv.isAlwaysNull());
    REQUIRE(dv.isNeverNull());

    dv.setNull(0);
    REQUIRE(dv.isAlwaysNull());
    REQUIRE_FALSE(dv.isNeverNull());
}

TEST_CASE("DictNullableVector setAll and setAllNull", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    dv.set(0, "a");
    dv.set(1, "b");
    dv.set(2, "c");

    dv.setAll("X");
    REQUIRE(dv.get(0) == "X");
    REQUIRE(dv.get(1) == "X");
    REQUIRE(dv.get(2) == "X");

    dv.setAllNull();
    REQUIRE(dv.isNull(0));
    REQUIRE(dv.isNull(1));
    REQUIRE(dv.isNull(2));
}

// ============================================================================
// Analytics
// ============================================================================

TEST_CASE("DictNullableVector distinctValues", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    dv.set(0, "alpha");
    dv.set(1, "beta");
    dv.set(2, "alpha");
    dv.set(3, "gamma");
    // index 4 null
    dv.set(5, "beta");

    auto vals = dv.distinctValues();
    REQUIRE(vals.size() == 3);
    REQUIRE(vals.count("alpha") == 1);
    REQUIRE(vals.count("beta") == 1);
    REQUIRE(vals.count("gamma") == 1);
}

TEST_CASE("DictNullableVector distinctValuesWithCounts", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    dv.set(0, "A");
    dv.set(1, "B");
    dv.set(2, "A");
    dv.set(3, "C");
    dv.set(4, "A");

    auto counts = dv.distinctValuesWithCounts();
    REQUIRE(counts["A"] == 3);
    REQUIRE(counts["B"] == 1);
    REQUIRE(counts["C"] == 1);
}

TEST_CASE("DictNullableVector minMaxValues", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    dv.set(0, "banana");
    dv.set(1, "apple");
    // index 2 null
    dv.set(3, "cherry");

    auto [valid, min_val, max_val] = dv.minMaxValues();
    REQUIRE(valid);
    REQUIRE(min_val == "apple");
    REQUIRE(max_val == "cherry");
}

TEST_CASE("DictNullableVector minMaxValues empty", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    auto [valid, min_val, max_val] = dv.minMaxValues();
    REQUIRE_FALSE(valid);
}

TEST_CASE("DictNullableVector distinctValuesWithIndexes range", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    dv.set(0, "X");
    dv.set(1, "Y");
    dv.set(2, "X");
    dv.set(3, "Z");

    auto vals = dv.distinctValuesWithIndexes(1, 3);
    REQUIRE(vals.size() == 3);
    REQUIRE(vals[std::string("Y")].size() == 1);
    REQUIRE(vals[std::string("Y")][0] == 1);
    REQUIRE(vals[std::string("X")].size() == 1);
    REQUIRE(vals[std::string("X")][0] == 2);
    REQUIRE(vals[std::string("Z")].size() == 1);
    REQUIRE(vals[std::string("Z")][0] == 3);
}

// ============================================================================
// Sorting
// ============================================================================

TEST_CASE("DictNullableVector sortPermutation", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    dv.set(0, "cherry");
    dv.set(1, "apple");
    dv.set(2, "banana");

    auto perm = dv.sortPermutation();
    // null < non-null, then lexicographic: apple(1), banana(2), cherry(0)
    REQUIRE(perm[0] == 1);
    REQUIRE(perm[1] == 2);
    REQUIRE(perm[2] == 0);
}

TEST_CASE("DictNullableVector sortByPermutation", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    dv.set(0, "cherry");
    dv.set(1, "apple");
    dv.set(2, "banana");

    auto perm = dv.sortPermutation();
    dv.sortByPermutation(perm);

    REQUIRE(dv.get(0) == "apple");
    REQUIRE(dv.get(1) == "banana");
    REQUIRE(dv.get(2) == "cherry");
}

TEST_CASE("DictNullableVector sortPermutation with nulls", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    dv.set(0, "B");
    // index 1 null
    dv.set(2, "A");

    dv.ensureMinSize(3);

    auto perm = dv.sortPermutation();
    dv.sortByPermutation(perm);

    // null sorts first
    REQUIRE(dv.isNull(0));
    REQUIRE(dv.get(1) == "A");
    REQUIRE(dv.get(2) == "B");
}

// ============================================================================
// Structural operations
// ============================================================================

TEST_CASE("DictNullableVector cutToSize", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    dv.set(0, "a");
    dv.set(1, "b");
    dv.set(2, "c");
    dv.set(3, "d");

    buf.cutToSize(2);
    REQUIRE(dv.contentSize() == 2);
    REQUIRE(dv.get(0) == "a");
    REQUIRE(dv.get(1) == "b");
}

TEST_CASE("DictNullableVector removeIndexes", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    dv.set(0, "A");
    dv.set(1, "B");
    dv.set(2, "C");
    dv.set(3, "D");
    dv.set(4, "E");

    std::vector<unsigned int> to_remove = {1, 3};
    buf.removeIndexes(to_remove);

    REQUIRE(dv.contentSize() == 3);
    REQUIRE(dv.get(0) == "A");
    REQUIRE(dv.get(1) == "C");
    REQUIRE(dv.get(2) == "E");
}

TEST_CASE("DictNullableVector addData merges dictionaries", "[dictvec]")
{
    Buffer buf1 = makeStrBuffer();
    auto& dv1 = buf1.get<std::string>("sv");
    dv1.set(0, "A");
    dv1.set(1, "B");

    Buffer buf2 = makeStrBuffer();
    auto& dv2 = buf2.get<std::string>("sv");
    dv2.set(0, "B");  // shared with dv1
    dv2.set(1, "C");  // new

    buf1.seizeBuffer(buf2);

    REQUIRE(buf1.size() == 4);
    REQUIRE(dv1.get(0) == "A");
    REQUIRE(dv1.get(1) == "B");
    REQUIRE(dv1.get(2) == "B");
    REQUIRE(dv1.get(3) == "C");
    // "B" should be shared in dictionary
    REQUIRE(dv1.dictionarySize() == 3); // A, B, C
}

// ============================================================================
// clearData / contentSize
// ============================================================================

TEST_CASE("DictNullableVector clearData", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    dv.set(0, "a");
    dv.set(1, "b");
    REQUIRE(dv.contentSize() == 2);

    dv.clearData();
    REQUIRE(dv.contentSize() == 0);
    REQUIRE(dv.dictionarySize() == 0);
    REQUIRE(dv.isNull(0));
}

// ============================================================================
// append (string concatenation with ";" separator)
// ============================================================================

TEST_CASE("DictNullableVector append concatenates", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    dv.set(0, "hello");
    dv.append(0, "world");
    REQUIRE(dv.get(0) == "hello;world");

    // original "hello" still in dictionary, plus "hello;world"
    REQUIRE(dv.dictionarySize() == 2);
}

// ============================================================================
// Dictionary efficiency -- high repetition scenario
// ============================================================================

TEST_CASE("DictNullableVector high repetition efficiency", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    // simulate 10000 rows with only 10 unique values
    const std::vector<std::string> values = {
        "AAL", "DLH", "BAW", "AFR", "KLM",
        "SAS", "AUA", "THY", "UAE", "SWR"
    };

    for (unsigned int i = 0; i < 10000; ++i)
        dv.set(i, values[i % values.size()]);

    REQUIRE(buf.size() == 10000);
    REQUIRE(dv.dictionarySize() == 10);
    REQUIRE(dv.contentSize() == 10000);

    // distinctValues is fast and correct
    auto distinct = dv.distinctValues();
    REQUIRE(distinct.size() == 10);

    // counts are correct
    auto counts = dv.distinctValuesWithCounts();
    for (auto& v : values)
        REQUIRE(counts[v] == 1000);

    // min/max
    auto [valid, min_v, max_v] = dv.minMaxValues();
    REQUIRE(valid);
    REQUIRE(min_v == "AAL");
    REQUIRE(max_v == "UAE");
}

TEST_CASE("DictNullableVector asJSON", "[dictvec]")
{
    Buffer buf = makeStrBuffer();
    auto& dv = buf.get<std::string>("sv");

    dv.set(0, "a");
    // index 1 null
    dv.set(2, "b");
    dv.ensureMinSize(3);

    auto j = dv.asJSON();
    REQUIRE(j.size() == 3);
    REQUIRE(j[0] == "a");
    REQUIRE(j[1].is_null());
    REQUIRE(j[2] == "b");
}
