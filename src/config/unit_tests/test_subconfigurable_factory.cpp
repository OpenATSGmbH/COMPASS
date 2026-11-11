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

#include "subconfigurablefactory.h"
#include "configurable.h"
#include "configuration.h"
#include "traced_assert.h"

#include "json.hpp"

#include <algorithm>
#include <memory>
#include <vector>

using nlohmann::json;

namespace
{

/**
*/
json makeConfig(const std::string& class_name,
                const std::string& instance_name)
{
    json cfg;
    cfg[ Configuration::CLASS_NAME_KEY    ] = class_name;
    cfg[ Configuration::INSTANCE_NAME_KEY ] = instance_name;

    return cfg;
}

/**
 * Base of the children created by the item factory.
 */
class ItemBase : public Configurable
{
public:
    ItemBase(json& cfg, Configurable* parent)
    :   Configurable(cfg, parent)
    {
        registerParameter("value", &value_, 0.0);
    }
    virtual ~ItemBase() = default;

    virtual std::string kind() const = 0;

    double value_ = 0.0;
};

class ItemA : public ItemBase
{
public:
    ItemA(json& cfg, Configurable* parent) : ItemBase(cfg, parent) {}
    std::string kind() const override final { return "A"; }
};

class ItemB : public ItemBase
{
public:
    ItemB(json& cfg, Configurable* parent, const std::string& tag) : ItemBase(cfg, parent), tag_(tag) {}
    std::string kind() const override final { return "B"; }

    std::string tag_;
};

class ItemC : public ItemBase
{
public:
    ItemC(json& cfg, Configurable* parent) : ItemBase(cfg, parent) {}
    std::string kind() const override final { return "C"; }
};

/**
 * Base of the children created by the second factory.
 */
class OtherItem : public Configurable
{
public:
    OtherItem(json& cfg, Configurable* parent) : Configurable(cfg, parent) {}
};

/**
 * Child created by the parent's own generateSubConfigurable() override.
 */
class PlainChild : public Configurable
{
public:
    PlainChild(json& cfg, Configurable* parent) : Configurable(cfg, parent) {}
};

/**
 * Parent with two static factories plus an own generateSubConfigurable() override.
 */
class FactoryParent : public Configurable
{
public:
    DECLARE_SUBCONFIGURABLE_FACTORY(itemFactory, ItemBase, FactoryParent, None)
    DECLARE_SUBCONFIGURABLE_FACTORY(otherFactory, OtherItem, FactoryParent, None)

    FactoryParent(json& cfg)
    :   Configurable(cfg, nullptr)
    {
        createSubConfigurables(); // factories attached by the macro members
    }

    ItemBase* item(const std::string& instance_name) const
    {
        auto it = std::find_if(items_.begin(), items_.end(),
                               [ & ] (const std::unique_ptr<ItemBase>& i) { return i->instanceName() == instance_name; });
        return it == items_.end() ? nullptr : it->get();
    }

    void deleteItem(const std::string& instance_name)
    {
        auto it = std::find_if(items_.begin(), items_.end(),
                               [ & ] (const std::unique_ptr<ItemBase>& i) { return i->instanceName() == instance_name; });
        traced_assert(it != items_.end());
        items_.erase(it);
    }

    std::vector<std::unique_ptr<ItemBase>>   items_;
    std::vector<std::unique_ptr<OtherItem>>  others_;
    std::vector<std::unique_ptr<PlainChild>> plain_;

    std::string tag_ = "parent_tag";

protected:
    void generateSubConfigurable(json& child_json) override
    {
        const auto& class_name = Configuration::getClassName(child_json);

        if (class_name == "PlainChild")
            plain_.emplace_back(new PlainChild(child_json, this));
        else
            throw std::runtime_error("FactoryParent: generateSubConfigurable: unknown class_name " + class_name);
    }

    void childCreated(Configurable* child) override
    {
        if (auto item = dynamic_cast<ItemBase*>(child))
        {
            items_.emplace_back(item);
            return;
        }

        auto other = dynamic_cast<OtherItem*>(child);
        traced_assert(other);

        others_.emplace_back(other);
    }
};

/**
 * Parent whose factory creates one default instance per class.
 */
class DefaultsParent : public Configurable
{
public:
    DECLARE_SUBCONFIGURABLE_FACTORY(itemFactory, ItemBase, DefaultsParent, OnePerClass)

    DefaultsParent(json& cfg)
    :   Configurable(cfg, nullptr)
    {
        createSubConfigurables();
    }

    ItemBase* item(const std::string& instance_name) const
    {
        auto it = std::find_if(items_.begin(), items_.end(),
                               [ & ] (const std::unique_ptr<ItemBase>& i) { return i->instanceName() == instance_name; });
        return it == items_.end() ? nullptr : it->get();
    }

    std::vector<std::unique_ptr<ItemBase>> items_;

protected:
    void childCreated(Configurable* child) override
    {
        auto item = dynamic_cast<ItemBase*>(child);
        traced_assert(item);

        items_.emplace_back(item);
    }
};

/**
*/
FactoryParent::itemFactoryType& FactoryParent::itemFactory()
{
    static itemFactoryType factory = { { "ItemA", &itemFactoryType::createDefault<ItemA>, "Item A" },
                                       { "ItemB", [ ] (json& cfg, FactoryParent& parent)
                                                  {
                                                      return std::unique_ptr<ItemBase>(new ItemB(cfg, &parent, parent.tag_));
                                                  }, "" } };
    return factory;
}

/**
*/
FactoryParent::otherFactoryType& FactoryParent::otherFactory()
{
    static otherFactoryType factory = { { "OtherItem", &otherFactoryType::createDefault<OtherItem>, "" } };
    return factory;
}

/**
*/
DefaultsParent::itemFactoryType& DefaultsParent::itemFactory()
{
    static itemFactoryType factory = { { "ItemA", &itemFactoryType::createDefault<ItemA>, "" } };
    return factory;
}

/**
 * Adds ItemC at runtime like a plugin would, once per process.
 */
void registerItemC()
{
    if (!FactoryParent::itemFactory().canCreate("ItemC"))
        FactoryParent::itemFactory().add<ItemC>("ItemC", "Item C");
    if (!DefaultsParent::itemFactory().canCreate("ItemC"))
        DefaultsParent::itemFactory().add<ItemC>("ItemC", "Item C");
}

} // anonymous namespace

TEST_CASE("SubConfigurableFactory entries", "[configurable][subconfigurablefactory]")
{
    auto& factory = FactoryParent::itemFactory();

    REQUIRE(factory.canCreate("ItemA"));
    REQUIRE(factory.canCreate("ItemB"));
    REQUIRE_FALSE(factory.canCreate("OtherItem"));
    REQUIRE_FALSE(factory.canCreate("PlainChild"));

    REQUIRE(factory.displayName("ItemA") == "Item A");
    REQUIRE(factory.displayName("ItemB") == "ItemB"); // no display name => class name

    auto names = factory.classNames();
    REQUIRE(std::find(names.begin(), names.end(), "ItemA") != names.end());
    REQUIRE(std::find(names.begin(), names.end(), "ItemB") != names.end());
}

TEST_CASE("SubConfigurableFactory creation", "[configurable][subconfigurablefactory]")
{
    json cfg = makeConfig("FactoryParent", "FP0");
    cfg[ "sub_configs" ] =
    {
        { "ItemA",      { { "A0", { { "parameters", { { "value", 1.5 } } } } } } },
        { "ItemB",      { { "B0", json::object() } } },
        { "OtherItem",  { { "O0", json::object() } } },
        { "PlainChild", { { "P0", json::object() } } }
    };

    FactoryParent parent(cfg);

    SECTION("children are created from the config and handed to childCreated()")
    {
        REQUIRE(parent.items_.size()  == 2);
        REQUIRE(parent.others_.size() == 1);

        auto a = parent.item("A0");
        REQUIRE(a);
        REQUIRE(a->kind() == "A");
        REQUIRE(a->value_ == Approx(1.5));
        REQUIRE(a->parentConfigurable() == &parent);
        REQUIRE(a->getPath() == "FP0.A0");

        auto b = dynamic_cast<ItemB*>(parent.item("B0"));
        REQUIRE(b);
        REQUIRE(b->tag_ == "parent_tag"); // custom creator with extra constructor args

        REQUIRE(parent.hasSubConfigurable("OtherItem", "O0"));
    }

    SECTION("unknown class names fall back to generateSubConfigurable()")
    {
        REQUIRE(parent.plain_.size() == 1);
        REQUIRE(parent.hasSubConfigurable("PlainChild", "P0"));
    }

    SECTION("generateSubConfigurableFromConfig() goes through the factory")
    {
        parent.generateSubConfigurableFromConfig("ItemA", "A1");

        REQUIRE(parent.items_.size() == 3);
        REQUIRE(parent.item("A1"));
        REQUIRE(parent.hasSubConfigurable("ItemA", "A1"));
    }

    SECTION("deleting a child unregisters it from the parent")
    {
        parent.deleteItem("A0");

        REQUIRE(parent.items_.size() == 1);
        REQUIRE_FALSE(parent.hasSubConfigurable("ItemA", "A0"));
        REQUIRE(parent.hasSubConfigurable("ItemB", "B0"));
    }
}

TEST_CASE("SubConfigurableFactory runtime registration", "[configurable][subconfigurablefactory]")
{
    registerItemC();

    REQUIRE(FactoryParent::itemFactory().canCreate("ItemC"));
    REQUIRE(FactoryParent::itemFactory().displayName("ItemC") == "Item C");

    json cfg = makeConfig("FactoryParent", "FP0");
    cfg[ "sub_configs" ] = { { "ItemC", { { "C0", json::object() } } } };

    FactoryParent parent(cfg);

    REQUIRE(parent.items_.size() == 1);
    REQUIRE(parent.item("C0"));
    REQUIRE(parent.item("C0")->kind() == "C");
}

TEST_CASE("SubConfigurableFactory default instances", "[configurable][subconfigurablefactory]")
{
    registerItemC();

    SECTION("None creates no default instances")
    {
        json cfg = makeConfig("FactoryParent", "FP0");
        FactoryParent parent(cfg);

        REQUIRE(parent.items_.empty());
        REQUIRE(parent.others_.empty());
    }

    SECTION("OnePerClass creates <class>0 for every class, incl. runtime-added ones")
    {
        json cfg = makeConfig("DefaultsParent", "DP0");
        DefaultsParent parent(cfg);

        REQUIRE(parent.items_.size() == 2);
        REQUIRE(parent.item("ItemA0"));
        REQUIRE(parent.item("ItemC0"));
    }

    SECTION("OnePerClass keeps existing children, whatever their instance name")
    {
        json cfg = makeConfig("DefaultsParent", "DP0");
        cfg[ "sub_configs" ] = { { "ItemA", { { "MyA", { { "parameters", { { "value", 3.0 } } } } } } } };

        DefaultsParent parent(cfg);

        REQUIRE(parent.items_.size() == 2);
        REQUIRE(parent.item("MyA")->value_ == Approx(3.0));
        REQUIRE_FALSE(parent.item("ItemA0"));
        REQUIRE(parent.item("ItemC0"));
    }
}
