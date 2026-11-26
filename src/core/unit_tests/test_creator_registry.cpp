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

#include "creatorregistry.h"

#include <functional>
#include <memory>
#include <string>

namespace
{

enum class Shape
{
    Circle,
    Square,
    Triangle
};

/**
 * Owner of a registry, declared like the extension points in the host.
 */
class ShapeOwner
{
public:
    typedef std::function<std::unique_ptr<std::string>(int size)> ShapeCreator;

    DECLARE_CREATOR_REGISTRY(shapeRegistry, Shape, ShapeCreator)
};

/**
*/
ShapeOwner::shapeRegistryType& ShapeOwner::shapeRegistry()
{
    static shapeRegistryType registry;
    return registry;
}

/**
*/
std::unique_ptr<std::string> createCircle(int size)
{
    return std::unique_ptr<std::string>(new std::string("circle " + std::to_string(size)));
}

} // namespace

TEST_CASE("CreatorRegistry registers and creates by key", "[util][registry]")
{
    auto& registry = ShapeOwner::shapeRegistry();

    // once per process, the registry is static
    if (!registry.has(Shape::Circle))
        registry.add(Shape::Circle, &createCircle);

    REQUIRE(registry.has(Shape::Circle));
    REQUIRE_FALSE(registry.has(Shape::Square));

    auto shape = registry.creator(Shape::Circle)(3);
    REQUIRE(shape);
    REQUIRE(*shape == "circle 3");

    const auto keys = registry.keys();
    REQUIRE(keys.size() == 1);
    REQUIRE(keys[ 0 ] == Shape::Circle);
}
