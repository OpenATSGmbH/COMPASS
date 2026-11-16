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

#pragma once

#include "json_fwd.hpp"

/**
 * View that accepts geo images exported by other views (e.g. the grid view's export).
 * - implemented by views next to View (e.g. GeographicView), found via ViewManager::viewsOfType<GeoImageSink>()
 */
class GeoImageSink
{
public:
    virtual ~GeoImageSink() = default;

    /// adds an annotation (view point annotation json, e.g. holding a geo image feature) as internal layer
    virtual void addInternalAnnotation(const nlohmann::json& annotation) = 0;
};
