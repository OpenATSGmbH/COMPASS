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

#include "dbcontent/variable/variableset.h"

#include <boost/date_time/posix_time/posix_time_duration.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

class Buffer;
class ViewableDataConfig;
class QWidget;

/**
 * The view functions that code below the view layer uses (injected via setManagers(...), e.g.
 * DBContentManager::viewProvider()). Implemented by ViewManager; its signals are connected to
 * the lower layers by CompassRoot.
 */
class IViewProvider
{
public:
    virtual ~IViewProvider() = default;

    virtual bool hasPendingViewProcessing() const = 0;
    virtual bool isProcessingData() const = 0;

    virtual void clearDataInViews() = 0;
    virtual dbContent::VariableSet getReadSet(const std::string& dbcontent_name) = 0;

    virtual void setCurrentViewPoint(std::shared_ptr<ViewableDataConfig> viewable,
                                     bool load_blocking = false) = 0;
    virtual void storeSelectedRecNums(const std::vector<unsigned long>& selected) = 0;

    virtual bool hasMaxLatency() const = 0;
    virtual boost::posix_time::time_duration maxLatency() const = 0;

    virtual const std::map<std::string, std::shared_ptr<Buffer>>& currentBuffers() const = 0;

    // view widget lookup, used by the rtcommand helpers to navigate the view tree
    virtual QWidget* viewContainerWidget(const std::string& name) = 0;
    virtual QWidget* latestViewContainerWidget() = 0;
    virtual QWidget* latestViewWidget() = 0;
};
