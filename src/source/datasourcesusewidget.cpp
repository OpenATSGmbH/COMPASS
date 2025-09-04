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
    : ds_man_(COMPASS::instance().dataSourceManager()),
      get_use_dstype_func_(get_use_dstype_func),
      set_use_dstype_func_(set_use_dstype_func),
      get_use_ds_func_(get_use_ds_func),
      set_use_ds_func_(set_use_ds_func),
      get_use_ds_line_func_(get_use_ds_line_func),
      set_use_ds_line_func_(set_use_ds_line_func)
{
    QFont font_bold;
    font_bold.setBold(true);

    QVBoxLayout* sub_layout = new QVBoxLayout();
    sub_layout->setContentsMargins(0, 0, 0, 0);

    QScrollArea* scroll_area = new QScrollArea();
    scroll_area->setWidgetResizable(true);
    scroll_area->setContentsMargins(0, 0, 0, 0);
//    scroll_area->setStyleSheet(" QAbstractScrollArea { background-color: transparent; }"
//                               " QWidget#scrollAreaWidgetContents{ background-color: transparent; }");

    sub_layout->addWidget(scroll_area);

    QWidget* sub_widget = new QWidget();
    sub_widget->setContentsMargins(0, 0, 0, 0);
    //sub_widget->setStyleSheet("background-color: white"); // works but overrides line colors

    scroll_area->setWidget(sub_widget);

    QVBoxLayout* main_layout = new QVBoxLayout();
    //main_layout->setContentsMargins(1, 1, 1, 1);

    // button

    QHBoxLayout* button_layout = new QHBoxLayout();
    //button_layout->setContentsMargins(0, 0, 0, 0);

    button_layout->addStretch();

    QPushButton* edit_button = new QPushButton();
    edit_button->setIcon(Files::IconProvider::getIcon("edit.png"));
    edit_button->setFixedSize(UI_ICON_SIZE);
    edit_button->setFlat(UI_ICON_BUTTON_FLAT);
    edit_button->setToolTip(tr("Data Source Options"));
    connect (edit_button, &QPushButton::clicked, this, &DataSourcesUseWidget::editClickedSlot);
    button_layout->addWidget(edit_button);

    edit_button->setDisabled(true);

    main_layout->addLayout(button_layout);

//    main_layout->addLayout(vlay);

//    main_layout->addStretch();

//    QHBoxLayout* vlay = new QHBoxLayout();
//    vlay->setContentsMargins(0, 0, 0, 0);

    // data sources, per type

    type_layout_ = new QGridLayout();

    main_layout->addLayout(type_layout_);

    main_layout->addStretch();

    updateContent();

    sub_widget->setLayout(main_layout);

    setLayout(sub_layout);

    // menu
    QAction* sel_dstyp_action = edit_menu_.addAction("Select All DSTypes");
    connect(sel_dstyp_action, &QAction::triggered, this, &DataSourcesUseWidget::selectAllDSTypesSlot);

    QAction* desel_dstyp_action = edit_menu_.addAction("Deselect All DSTypes");
    connect(desel_dstyp_action, &QAction::triggered, this, &DataSourcesUseWidget::deselectAllDSTypesSlot);

    edit_menu_.addSeparator();

    QMenu* select_ds = edit_menu_.addMenu("Select Data Sources");

    QAction* sel_ds_action = select_ds->addAction("All");
    connect(sel_ds_action, &QAction::triggered, this, &DataSourcesUseWidget::selectAllDataSourcesSlot);

    for (const auto& ds_type_it : ds_man_.data_source_types_)
    {
        QAction* action = select_ds->addAction(("From "+ds_type_it).c_str());
        action->setProperty("ds_type", ds_type_it.c_str());
        connect(action, &QAction::triggered, this, &DataSourcesUseWidget::selectDSTypeSpecificDataSourcesSlot);
    }

    QMenu* deselect_ds = edit_menu_.addMenu("Deselect Data Sources");

    QAction* desel_ds_action = deselect_ds->addAction("All");
    connect(desel_ds_action, &QAction::triggered, this, &DataSourcesUseWidget::deselectAllDataSourcesSlot);


    for (const auto& ds_type_it : ds_man_.data_source_types_)
    {
        QAction* action = deselect_ds->addAction(("From "+ds_type_it).c_str());
        action->setProperty("ds_type", ds_type_it.c_str());
        connect(action, &QAction::triggered, this, &DataSourcesUseWidget::deselectDSTypeSpecificDataSourcesSlot);
    }

    edit_menu_.addSeparator();

    QMenu* set_lines = edit_menu_.addMenu("Set Line");

    QAction* desel_line_action = set_lines->addAction("Deselect All");
    connect(desel_line_action, &QAction::triggered, this, &DataSourcesUseWidget::deselectAllLinesSlot);

    for (unsigned int cnt=0; cnt < 4; ++cnt)
    {
        QAction* desel_line_action = set_lines->addAction(("Select "+String::lineStrFrom(cnt)).c_str());
        desel_line_action->setProperty("line_id", cnt);
        connect(desel_line_action, &QAction::triggered, this, &DataSourcesUseWidget::selectSpecificLineSlot);
    }

}

DataSourcesUseWidget::~DataSourcesUseWidget()
{
}

void DataSourcesUseWidget::disableDataSources (const std::set<unsigned int>& disabled_ds)
{
    for (auto ds_id : disabled_ds)
    {
        set_use_ds_func_(ds_id, false);

        if (ds_widgets_.count(ds_id))
        {
            ds_widgets_.at(ds_id)->setLoadChecked(false);
            ds_widgets_.at(ds_id)->setDisabled(true);
        }
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

//void DataSourcesUseWidget::loadDSChangedSlot()
//{
//    QCheckBox* box = dynamic_cast<QCheckBox*>(QObject::sender());
//    traced_assert(box);

//    unsigned int ds_id = box->property("DS ID").toUInt();

//    bool load = box->checkState() == Qt::Checked;

//    loginf << "ds_id " << ds_id << " load " << load;

//    //ds_man_.dbDataSource(ds_id).loadingWanted(load);
//    set_use_ds_func_(ds_id, load);
//}

void DataSourcesUseWidget::editClickedSlot()
{
    loginf;

    edit_menu_.exec(QCursor::pos());
}

void DataSourcesUseWidget::selectAllDSTypesSlot()
{
    loginf;

    ds_man_.selectAllDSTypes();

}
void DataSourcesUseWidget::deselectAllDSTypesSlot()
{
    loginf;

    ds_man_.deselectAllDSTypes();
}

void DataSourcesUseWidget::selectAllDataSourcesSlot()
{
    loginf;

    ds_man_.selectAllDataSources();
}
void DataSourcesUseWidget::deselectAllDataSourcesSlot()
{
    loginf;

    ds_man_.deselectAllDataSources();
}

void DataSourcesUseWidget::selectDSTypeSpecificDataSourcesSlot()
{
    QAction* action = dynamic_cast<QAction*>(sender());
    traced_assert(action);

    string ds_type = action->property("ds_type").toString().toStdString();

    loginf << "ds_type '" << ds_type << "'";

    ds_man_.selectDSTypeSpecificDataSources(ds_type);
}

void DataSourcesUseWidget::deselectDSTypeSpecificDataSourcesSlot()
{
    QAction* action = dynamic_cast<QAction*>(sender());
    traced_assert(action);

    string ds_type = action->property("ds_type").toString().toStdString();

    loginf << "ds_type '" << ds_type << "'";

    ds_man_.deselectDSTypeSpecificDataSources(ds_type);
}

void DataSourcesUseWidget::deselectAllLinesSlot()
{
    loginf;

    ds_man_.deselectAllLines();
}

void DataSourcesUseWidget::selectSpecificLineSlot()
{
    QAction* action = dynamic_cast<QAction*>(sender());
    traced_assert(action);

    unsigned int line_id = action->property("line_id").toUInt();

    loginf << "line_id " << line_id;

    ds_man_.selectSpecificLineSlot(line_id);
}

void DataSourcesUseWidget::updateContent()
{
    logdbg << "num data sources " << ds_man_.dbDataSources().size();

    bool recreate_required = false;

    for (auto& ds_type_it : ds_type_boxes_)
        ds_type_it.second->setChecked(get_use_dstype_func_(ds_type_it.first));
    //ds_man_.dsTypeLoadingWanted(ds_type_it.first));

    if (ds_widgets_.size() != ds_man_.dbDataSources().size()) // check if same size
        recreate_required = true;
    else // check each one
    {
        for (const auto& ds_it : ds_man_.dbDataSources())
        {
            if (!ds_widgets_.count(ds_it->id()))
            {
                logdbg << "ds_box " << ds_it->name() << " missing ";

                recreate_required = true;
                break;
            }
        }
    }

    logdbg << "recreate_required " << recreate_required;

    if (recreate_required)
    {
        clearAndCreateContent();
    }
    else
    {
        for (auto& ds_widget_it : ds_widgets_)
            ds_widget_it.second->updateContent();
    }

    arrangeSourceWidgetWidths();
}

void DataSourcesUseWidget::clear()
{
    // delete dstype boxes
    for (auto& ds_type_box_it : ds_type_boxes_)
        delete ds_type_box_it.second;
    ds_type_boxes_.clear();

    ds_widgets_.clear(); // delete by data sources

            // delete all previous
    QLayoutItem* child;
    while (!type_layout_->isEmpty() && (child = type_layout_->takeAt(0)) != nullptr)
    {
        if (child->widget())
            delete child->widget();
        delete child;
    }
}

void DataSourcesUseWidget::arrangeSourceWidgetWidths()
{
    logdbg;

    unsigned int min_width = 0;

    for (auto& widget_it : ds_widgets_)
        min_width = max(min_width, widget_it.second->getLabelMinWidth());

    if (min_width)
    {
        logdbg << "setting width " << min_width;

        for (auto& widget_it : ds_widgets_)
            widget_it.second->updateLabelMinWidth(min_width);
    }
}

void DataSourcesUseWidget::clearAndCreateContent()
{
    logdbg;

    clear();

    QFont font_bold;
    font_bold.setBold(true);
    font_bold.setPointSize(ds_man_.config().ds_font_size_);

    unsigned int row = 0;
    unsigned int col = 0;
    //unsigned int dstype_col = 0;
    unsigned int dstyp_cnt = 0;

    unsigned int ds_id;
    string ds_name;
    string ds_content_name;

    using namespace dbContent;

    //DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();
    bool ds_found;

    logdbg << "iterating data source types";

    for (auto& ds_type_name : DataSourceManager::data_source_types_)
    {
        logdbg << "typ " << ds_type_name << " cnt " << dstyp_cnt;

        if (ds_type_name == "MLAT" || ds_type_name == "Tracker")  // break into next column
        {
            row = 0;
            col++;
        }

        QCheckBox* dstyp_box = new QCheckBox(ds_type_name.c_str());
        dstyp_box->setFont(font_bold);
        dstyp_box->setChecked(get_use_dstype_func_(ds_type_name));//ds_man_.dsTypeLoadingWanted(ds_type_name));
        dstyp_box->setProperty("DSType", ds_type_name.c_str());

        connect(dstyp_box, &QCheckBox::clicked,
                this, &DataSourcesUseWidget::loadDSTypeChangedSlot);

        dstyp_box->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);

        type_layout_->addWidget(dstyp_box, row, col);

        traced_assert(!ds_type_boxes_.count(ds_type_name));
        ds_type_boxes_[ds_type_name] = dstyp_box;

        ++row;

        ds_found = false;

        logdbg << "data sources for type " << ds_type_name;

        for (const auto& ds_it : ds_man_.dbDataSources())
        {
            if (ds_it->dsType() != ds_type_name)
                continue;

            ds_found = true;

            ds_id = Number::dsIdFrom(ds_it->sac(), ds_it->sic());
            ds_name = ds_it->name();
            logdbg << "create '" << ds_it->dsType()
                   << "' '" << ds_name << "'";

            std::function<bool()> get_use_ds_func =
                [this,ds_id] () { return get_use_ds_func_(ds_id); };
            std::function<void(bool)> set_use_ds_func  =
                [this,ds_id] (bool value) { return set_use_ds_func_(ds_id, value); };
            std::function<bool(unsigned int)> get_use_ds_line_func =
                [this,ds_id] (unsigned int line_id) { return get_use_ds_line_func_(ds_id,line_id); };
            std::function<void(unsigned int, bool)> set_use_ds_line_func  =
                [this,ds_id] (unsigned int line_id, bool value) { return set_use_ds_line_func_(ds_id,line_id, value); };

            std::function<bool()> show_counts_func = [] () { return false; };

            DBDataSourceWidget* ds_widget = new DBDataSourceWidget(
                *ds_it, get_use_ds_func, set_use_ds_func,
                get_use_ds_line_func, set_use_ds_line_func,
                show_counts_func);

            type_layout_->addWidget(ds_widget, row, col);

            traced_assert(!ds_widgets_.count(ds_id));
            ds_widgets_[ds_id] = ds_widget;

            ++row;
        }

        if (!ds_found)
        {
            dstyp_box->setChecked(false);
            dstyp_box->setDisabled(true);
        }

        dstyp_cnt++;
    }

    logdbg << "setting columns";

    for(int c=0; c < type_layout_->columnCount(); c++)
        type_layout_->setColumnStretch(c,1);

    logdbg << "done";
}



