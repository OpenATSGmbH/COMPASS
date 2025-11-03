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

#include "viewinfo.h"

#include "traced_assert.h"

/**
*/
void ViewInfos::clear()
{
    infos_.clear();

    num_infos_    = 0;
    num_sections_ = 0;
}

/**
*/
ViewInfos& ViewInfos::addSection(const std::string& name)
{
    traced_assert(!name.empty());

    infos_.emplace_back("", name, "", true, Style::Bold);
    ++num_sections_;

    return *this;
}

/**
*/
ViewInfos& ViewInfos::addInfo(const std::string& id,
                              const std::string& name, 
                              const std::string& value,
                              bool value_italic)
{
    traced_assert(!name.empty() || !value.empty());

    infos_.emplace_back(id, name, value, false, value_italic ? Style::Italic : Style::None);
    ++num_infos_;

    return *this;
}

/**
*/
ViewInfos& ViewInfos::addSpace()
{
    infos_.emplace_back("", "", "", false);

    return *this;
}

/**
 */
ViewInfos& ViewInfos::addInfos(const ViewInfos& other)
{
    infos_.insert(infos_.end(), other.infos().begin(), other.infos().end());

    num_sections_ += other.numSections();
    num_infos_    += other.numInfos();

    return *this;
}

/**
*/
const std::vector<ViewInfos::ViewInfo>& ViewInfos::infos() const
{
    return infos_;
}

/**
*/
size_t ViewInfos::numSections() const
{
    return num_sections_;
}

/**
*/
size_t ViewInfos::numInfos() const
{
    return num_infos_;
}
