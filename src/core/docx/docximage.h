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

#include "docxcontent.h"

#include <string>

class DocxImage : public DocxContent
{
public:
    DocxImage(const std::string& filename, const std::string& caption,
              const std::string& rel_id, int width_emu, int height_emu);

    std::string toXml() override;

    std::string filename() const;
    std::string relId() const;

protected:
    std::string filename_;
    std::string caption_;
    std::string rel_id_;
    int width_emu_;
    int height_emu_;
};
