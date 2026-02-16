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

#include "ui_test_common.h"

#include "json_fwd.hpp"

#include <boost/optional.hpp>

#include <QString>

class QWidget;

namespace ui_test
{

    bool setUIElementNative(QWidget* parent, 
                            const QString& obj_name, 
                            const QString& value, 
                            int delay = -1,
                            const SetUIHint& hint = SetUIHint());

    boost::optional<QString> getUIElementNative(QWidget* parent, 
                                                const QString& obj_name,
                                                const QString& what);
    nlohmann::json getUIElementNativeJSON(QWidget* parent, 
                                          const QString& obj_name,
                                          const QString& what);

} // namespace ui_test
