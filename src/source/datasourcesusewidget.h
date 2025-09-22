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

#include "datasourceswidget.h"

#include <QWidget>
#include <QMenu>

#include <map>
#include <set>
#include <functional>

class DataSourceManager;

class QGridLayout;
class QPushButton;
class QCheckBox;
class QLineEdit;
class QLabel;

namespace dbContent
{
class DBDataSourceWidget;
}

class DataSourcesUseWidget : public DataSourcesWidget
{
    Q_OBJECT

  public slots:
    void loadDSTypeChangedSlot();

    // void editClickedSlot();

  public:
    DataSourcesUseWidget(std::function<bool(const std::string&)> get_use_dstype_func,
                         std::function<void(const std::string&,bool)> set_use_dstype_func,
                         std::function<bool(unsigned int)> get_use_ds_func,
                         std::function<void(unsigned int,bool)> set_use_ds_func,
                         std::function<bool(unsigned int,unsigned int)> get_use_ds_line_func,
                         std::function<void(unsigned int,unsigned int, bool)> set_use_ds_line_func);
    virtual ~DataSourcesUseWidget();

    void disableDataSources (const std::set<unsigned int>& disabled_ds);

    void loadingDone();

  private:
    std::function<bool(const std::string&)> get_use_dstype_func_;
    std::function<void(const std::string&, bool)> set_use_dstype_func_;

    std::function<bool(unsigned int)> get_use_ds_func_;
    std::function<void(unsigned int,bool)> set_use_ds_func_;

    std::function<bool(unsigned int,unsigned int)> get_use_ds_line_func_;
    std::function<void(unsigned int,unsigned int, bool)> set_use_ds_line_func_;
    
    virtual void setUseDSType(const std::string& ds_type_name, bool use) override;
    virtual bool getUseDSType(const std::string& ds_type_name) const override;
    virtual void setUseDS(unsigned int ds_id, bool use) override;
    virtual bool getUseDS(unsigned int ds_id) const override;
    virtual void setUseDSLine(unsigned int ds_id, unsigned int ds_line, bool use) override;
    virtual bool getUseDSLine(unsigned int ds_id, unsigned int ds_line) const override;
    virtual void setShowCounts(bool show) const override;
    virtual bool getShowCounts() const override;

    //void clearAndCreateContent();

    //void clear();
    //void arrangeSourceWidgetWidths();
};

