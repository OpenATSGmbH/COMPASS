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

#include <cstdint>
#include "datasourceswidgetbase.h"
#include "datasourcesstatus.h"

class SensorStatusTracker;
class DataSourcesStatusWidget;
class DataSourcesWidgetItemBase;

class QLabel;

/**
 */
class DataSourceStatusItem : public DataSourceItemBase
{
public:
    DataSourceStatusItem(DataSourcesStatusWidget* widget,
                         DataSourcesWidgetItemBase* parent);

    virtual ~DataSourceStatusItem() = default;

protected:
    virtual void init_impl() override;
    virtual void updateContentChanges_impl() override;
    virtual void updateContent_impl() override;

private:
    void showStatus(const std::string& msg, const QColor& color);

    DataSourcesStatusWidget* status_widget_ = nullptr;
    QLabel*                  status_label_  = nullptr;
};

/**
 * Shows the sensor status of the active tracker line, as tracked by the SensorStatusTracker.
 * - updated on the tracker's signals
 */
class DataSourcesStatusWidget : public DataSourcesWidgetBase
{
    Q_OBJECT

public:
    DataSourcesStatusWidget(context::DBContextManager& ctx_man,
                            SensorStatusTracker& tracker,
                            bool init_ui = true);
    virtual ~DataSourcesStatusWidget();

    SensorStatusTracker& tracker() { return tracker_; }
    const SensorStatusTracker& tracker() const { return tracker_; }

    void reset();

    void showLastUpdates(bool show);

    static QColor colorFromSensorStatus(sensor_status::SensorStatus status);
    static QColor colorFromEvent(const sensor_status::Event& evt);
    static QFont  fontFromEvent(const sensor_status::Event& evt);

    static const QColor ColorConOperational;
    static const QColor ColorConDegraded;
    static const QColor ColorConInitialization;
    static const QColor ColorConDisconnected;
    static const QColor ColorCoasting;
    static const QColor ColorStatusUnknown;
    static const QColor ColorStatusInfoMissing;
    static const QColor ColorNoData;

    static const int StatusColumn;
    static const int LastUpdateColumn;

protected:
    virtual QStringList getCustomColumnHeaders() const override;

    virtual bool showDSType(const std::string& ds_type_name) const override;
    virtual bool showDS(unsigned int ds_id) const override;

    virtual DataSourceItemBase* createDSItem(DataSourcesWidgetItemBase* parent = nullptr) override;

private:
    SensorStatusTracker& tracker_;
};
