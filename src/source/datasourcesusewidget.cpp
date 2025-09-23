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

#include "datasourcesusewidget.h"
#include "compass.h"
#include "dbcontent/dbcontentmanager.h"
#include "source/dbdatasourcewidget.h"
#include "datasourcemanager.h"
#include "datasourceeditwidget.h"
#include "global.h"
#include "stringconv.h"
#include "number.h"
#include "files.h"

#include <QCheckBox>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QScrollArea>

using namespace std;
using namespace Utils;

DataSourcesUseWidget::DataSourcesUseWidget(std::function<bool(const std::string&)> get_use_dstype_func,
                         std::function<void(const std::string&,bool)> set_use_dstype_func,
                         std::function<bool(unsigned int)> get_use_ds_func,
                         std::function<void(unsigned int,bool)> set_use_ds_func,
                         std::function<bool(unsigned int,unsigned int)> get_use_ds_line_func,
                         std::function<void(unsigned int,unsigned int, bool)> set_use_ds_line_func)
    : DataSourcesWidget(false, COMPASS::instance().dataSourceManager()),
    get_use_dstype_func_(get_use_dstype_func), set_use_dstype_func_(set_use_dstype_func),
    get_use_ds_func_(get_use_ds_func), set_use_ds_func_(set_use_ds_func),
    get_use_ds_line_func_(get_use_ds_line_func), set_use_ds_line_func_(set_use_ds_line_func)
{
    assert (tree_widget_);
    tree_widget_->setMaximumWidth(400);

    std::function<void(unsigned int)> update_ds_func =
        [this] (unsigned int ds_id) { this->updateContent(false); };

    std::function<void(unsigned int)> delete_ds_func = [this](unsigned int ds_id)
    { this->updateContent(true); };

    assert (top_layout_);
    edit_widget_ = new DataSourceEditWidget (ds_man_, update_ds_func, delete_ds_func);
    edit_widget_->setContentsMargins(0, 0, 0, 0);
    
    top_layout_->addWidget(edit_widget_);    

    // edit_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    // top_layout_->setStretchFactor(tree_widget_, 0);
    // top_layout_->setStretchFactor(edit_widget_, 1);
    
    connect(this, &DataSourcesUseWidget::dataSourceSelectedSignal, this,
            &DataSourcesUseWidget::dataSourceSelectedSlot);
}

DataSourcesUseWidget::~DataSourcesUseWidget()
{
}

void DataSourcesUseWidget::disableDataSources (const std::set<unsigned int>& disabled_ds)
{
    for (auto ds_id : disabled_ds)
    {
        set_use_ds_func_(ds_id, false);

        setUseDS(ds_id, false);
    }
}


void DataSourcesUseWidget::loadDSTypeChangedSlot()
{
    QCheckBox* box = dynamic_cast<QCheckBox*>(QObject::sender());
    traced_assert(box);

    string ds_type_name =box->property("DSType").toString().toStdString();

    bool load = box->checkState() == Qt::Checked;

    loginf << "ds_type " << ds_type_name << " load " << load;

    //COMPASS::instance().dataSourceManager().dsTypeLoadingWanted(ds_type_name, load);
    set_use_dstype_func_(ds_type_name, load);

    box->setChecked(get_use_dstype_func_(ds_type_name));
}

void DataSourcesUseWidget::dataSourceSelectedSlot(unsigned int ds_id)
{
    loginf << " ds_id " << ds_id;

    assert (edit_widget_);
    edit_widget_->showID(ds_id);
}

void DataSourcesUseWidget::setUseDSType(const std::string& ds_type_name, bool use)
{
    set_use_dstype_func_(ds_type_name, use);
}
bool DataSourcesUseWidget::getUseDSType(const std::string& ds_type_name) const
{
    return get_use_dstype_func_(ds_type_name);
}
void DataSourcesUseWidget::setUseDS(unsigned int ds_id, bool use) { set_use_ds_func_(ds_id, use); }
bool DataSourcesUseWidget::getUseDS(unsigned int ds_id) const { return get_use_ds_func_(ds_id); }
void DataSourcesUseWidget::setUseDSLine(unsigned int ds_id, unsigned int ds_line, bool use)
{
    set_use_ds_line_func_(ds_id, ds_line, use);
}
bool DataSourcesUseWidget::getUseDSLine(unsigned int ds_id, unsigned int ds_line) const
{
    return get_use_ds_line_func_(ds_id, ds_line);
}
void DataSourcesUseWidget::setShowCounts(bool show) const {}
bool DataSourcesUseWidget::getShowCounts() const { return false; }
