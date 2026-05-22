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

#include "configurable.h"
#include "configuration.h"
#include "json.hpp"

using nlohmann::json;

namespace
{

/**
 * Build a minimal config JSON with class_name and instance_name embedded.
 */
static json makeConfig(const std::string& class_name,
                       const std::string& instance_name,
                       json params = json::object())
{
    json cfg;
    cfg[Configuration::CLASS_NAME_KEY]    = class_name;
    cfg[Configuration::INSTANCE_NAME_KEY] = instance_name;
    if (!params.empty())
        cfg["parameters"] = params;
    return cfg;
}

/**
 * Simple test Configurable with two parameters.
 */
class TestConfigurable : public Configurable
{
public:
    TestConfigurable(nlohmann::json& config, Configurable* parent = nullptr)
        : Configurable(config, parent)
    {
        registerParameter("threshold", &threshold_, 50.0);
        registerParameter("name", &name_, std::string("default"));
    }

    void checkSubConfigurables() override {}

    // Expose protected setParameter for testing
    template <typename T>
    void setParam(T& param, const T& value) { setParameter(param, value); }

    double threshold_{50.0};
    std::string name_{"default"};
};

/**
 * Test Configurable that creates TestConfigurable children from sub_configs.
 */
class TestParentConfigurable : public Configurable
{
public:
    TestParentConfigurable(nlohmann::json& config, Configurable* parent = nullptr)
        : Configurable(config, parent)
    {
        registerParameter("enabled", &enabled_, true);
        createSubConfigurables();
    }

    ~TestParentConfigurable() override
    {
        for (auto* child : owned_children_)
            delete child;
    }

    void generateSubConfigurable(nlohmann::json& child_json) override
    {
        const auto& class_name = Configuration::getClassName(child_json);
        if (class_name == "TestConfigurable")
        {
            auto* child = new TestConfigurable(child_json, this);
            owned_children_.push_back(child);
        }
    }

    void checkSubConfigurables() override {}

    bool enabled_{true};
    std::vector<TestConfigurable*> owned_children_;
};

} // anonymous namespace

TEST_CASE("Configurable construction", "[configurable]")
{
    SECTION("reads parameters from json")
    {
        json cfg = makeConfig("TestConfigurable", "TC0",
                              {{"threshold", 75.0}, {"name", "test_value"}});
        TestConfigurable tc(cfg);

        REQUIRE(tc.threshold_ == Approx(75.0));
        REQUIRE(tc.name_ == "test_value");
        REQUIRE_FALSE(tc.isTransient());
    }

    SECTION("uses defaults for missing parameters")
    {
        json cfg = makeConfig("TestConfigurable", "TC0");
        TestConfigurable tc(cfg);

        REQUIRE(tc.threshold_ == Approx(50.0));
        REQUIRE(tc.name_ == "default");
    }

    SECTION("uses defaults for empty parameters section")
    {
        json cfg = makeConfig("TestConfigurable", "TC0", json::object());
        TestConfigurable tc(cfg);

        REQUIRE(tc.threshold_ == Approx(50.0));
        REQUIRE(tc.name_ == "default");
    }

    SECTION("partial parameters - present values read, missing use defaults")
    {
        json cfg = makeConfig("TestConfigurable", "TC0", {{"threshold", 99.9}});
        TestConfigurable tc(cfg);

        REQUIRE(tc.threshold_ == Approx(99.9));
        REQUIRE(tc.name_ == "default");
    }
}

TEST_CASE("Configuration write-back", "[configurable]")
{
    SECTION("writeBackConfig updates backing json with current parameter values")
    {
        json cfg = makeConfig("TestConfigurable", "TC0",
                              {{"threshold", 10.0}, {"name", "original"}});
        TestConfigurable tc(cfg);

        REQUIRE(tc.threshold_ == Approx(10.0));
        REQUIRE(tc.name_ == "original");

        // Modify parameters directly
        tc.threshold_ = 42.0;
        tc.name_ = "modified";

        // Write back
        tc.writeBackConfig();

        // Verify the backing json was updated
        REQUIRE(cfg.contains("parameters"));
        REQUIRE(cfg["parameters"]["threshold"].get<double>() == Approx(42.0));
        REQUIRE(cfg["parameters"]["name"].get<std::string>() == "modified");
    }

    SECTION("writeBackConfig on non-json-backed is a no-op")
    {
        // Default-constructed Configurable is not json-backed
        Configurable c;
        c.writeBackConfig(); // should not crash
    }
}

TEST_CASE("parent-child", "[configurable]")
{
    SECTION("parent creates children from sub_configs")
    {
        json cfg = {
            {Configuration::CLASS_NAME_KEY,    "TestParent"},
            {Configuration::INSTANCE_NAME_KEY, "TP0"},
            {"parameters", {{"enabled", true}}},
            {"sub_configs", {
                {"TestConfigurable", {
                    {"Child0", {{"parameters", {{"threshold", 11.0}, {"name", "child_a"}}}}},
                    {"Child1", {{"parameters", {{"threshold", 22.0}, {"name", "child_b"}}}}}
                }}
            }}
        };

        TestParentConfigurable parent(cfg);

        REQUIRE(parent.enabled_ == true);
        REQUIRE(parent.owned_children_.size() == 2);

        // Find children by name (order depends on json iteration)
        TestConfigurable* child_a = nullptr;
        TestConfigurable* child_b = nullptr;
        for (auto* c : parent.owned_children_)
        {
            if (c->instanceName() == "Child0") child_a = c;
            if (c->instanceName() == "Child1") child_b = c;
        }

        REQUIRE(child_a != nullptr);
        REQUIRE(child_b != nullptr);
        REQUIRE(child_a->threshold_ == Approx(11.0));
        REQUIRE(child_a->name_ == "child_a");
        REQUIRE(child_b->threshold_ == Approx(22.0));
        REQUIRE(child_b->name_ == "child_b");
    }

    SECTION("addChild and removeChild manage children_vec")
    {
        json cfg_parent = makeConfig("TestParent", "TP0", {{"enabled", true}});
        json cfg_child  = makeConfig("TestConfigurable", "TC0", {{"threshold", 5.0}});

        TestParentConfigurable parent(cfg_parent);
        TestConfigurable child(cfg_child);

        parent.addChild(&child);
        // We can't directly inspect children_vec_, but we can verify
        // removeChild doesn't crash and the parent-child relationship works.
        parent.removeChild(&child);
        // Removing again should be safe (no-op)
        parent.removeChild(&child);
    }
}

TEST_CASE("getPath", "[configurable]")
{
    SECTION("root configurable path is just instance_name")
    {
        json cfg = makeConfig("TestConfigurable", "TC0");
        TestConfigurable tc(cfg);

        REQUIRE(tc.getPath() == "TC0");
    }

    SECTION("child path is parent_path.instance_name")
    {
        json cfg_parent = makeConfig("TestParent", "Parent0");
        json cfg_child  = makeConfig("TestConfigurable", "TC0");

        TestParentConfigurable parent(cfg_parent);
        TestConfigurable child(cfg_child, &parent);

        REQUIRE(child.getPath() == "Parent0.TC0");
    }

    SECTION("nested path propagates through parent_path")
    {
        json cfg_root = makeConfig("TestParent", "Root0");
        json cfg_mid  = makeConfig("TestParent", "Mid0");
        json cfg_leaf = makeConfig("TestConfigurable", "TC0");

        TestParentConfigurable root(cfg_root);
        TestParentConfigurable mid(cfg_mid, &root);
        TestConfigurable leaf(cfg_leaf, &mid);

        REQUIRE(leaf.getPath() == "Root0.Mid0.TC0");
    }

    SECTION("parent passes its path_str to children via createSubConfigurables")
    {
        json cfg = {
            {Configuration::CLASS_NAME_KEY,    "TestParent"},
            {Configuration::INSTANCE_NAME_KEY, "TP0"},
            {"parameters", {{"enabled", true}}},
            {"sub_configs", {
                {"TestConfigurable", {
                    {"Child0", {{"parameters", {{"threshold", 1.0}}}}}
                }}
            }}
        };

        json cfg_root = makeConfig("TestParent", "Root0");
        TestParentConfigurable root(cfg_root);
        TestParentConfigurable parent(cfg, &root);

        REQUIRE(parent.getPath() == "Root0.TP0");
        REQUIRE(parent.owned_children_.size() == 1);
        REQUIRE(parent.owned_children_[0]->getPath() == "Root0.TP0.Child0");
    }
}

TEST_CASE("notifyModifications propagation", "[configurable]")
{
    SECTION("notifyModifications propagates through parent")
    {
        // Create a parent that tracks onModified calls
        struct TrackingParent : public Configurable
        {
            TrackingParent(nlohmann::json& cfg)
                : Configurable(cfg, nullptr) {}
            void onModified() override { modify_count_++; }
            void checkSubConfigurables() override {}
            int modify_count_{0};
        };

        json cfg_parent = makeConfig("Tracker", "Tracker0");
        json cfg_child  = makeConfig("TestConfigurable", "TC0", {{"threshold", 5.0}});

        TrackingParent parent(cfg_parent);

        // Construct child with parent - parent_ is set in the constructor
        TestConfigurable child(cfg_child, &parent);

        child.setParam(child.threshold_, 20.0);
        REQUIRE(parent.modify_count_ == 1);

        child.setParam(child.threshold_, 30.0);
        REQUIRE(parent.modify_count_ == 2);

        // Same value - no notification
        child.setParam(child.threshold_, 30.0);
        REQUIRE(parent.modify_count_ == 2);
    }

    SECTION("no propagation without parent")
    {
        json cfg = makeConfig("TestConfigurable", "TC0", {{"threshold", 5.0}});
        TestConfigurable child(cfg);

        // Should not crash with nullptr parent
        child.setParam(child.threshold_, 10.0);
        REQUIRE(child.threshold_ == 10.0);
    }
}
