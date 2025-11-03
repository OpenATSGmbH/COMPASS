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

#include "report/treemodel.h"
#include "json_fwd.hpp"

#include <boost/optional.hpp>

#include <QWidget>
#include <QTreeView>
#include <QStackedWidget>

class TaskResultsWidget;
class PopupMenu;

class QPushButton;
class QLabel;

namespace ResultReport
{

class Section;

class ReportWidget : public QWidget
{
    Q_OBJECT

public slots:
    void itemClickedSlot(const QModelIndex& index);
    void itemDblClickedSlot(const QModelIndex& index);
    void contextMenuSlot(const QPoint& pos);

    void stepBackSlot();

public:
    ReportWidget(TaskResultsWidget& task_result_widget);
    virtual ~ReportWidget();

    void setReport(const std::shared_ptr<Report>& report);
    void clear();

    void expand();

    void showResultWidget(Section* section, // can be nullptr
                          bool preload_ondemand_contents); 

    void selectId (const std::string& id, 
                   bool show_figure,
                   const nlohmann::json& config);
    void reshowLastId ();

    std::string currentSectionID() const;
    nlohmann::json currentSectionConfig() const;

    void showFigure(const QModelIndex& index);

    static const std::string FieldConfigScrollPosV;
    static const std::string FieldConfigScrollPosH;

protected:
    void expandAllParents (QModelIndex index);
    void updateBackButton ();
    void updateCurrentSectionLabel();
    void triggerItem (const QModelIndex& index,
                      bool preload_ondemand_contents);
    
    bool configureSection(const nlohmann::json& config);

    TaskResultsWidget& task_result_widget_;

    ResultReport::TreeModel tree_model_;
    QTreeView* tree_view_;

    QStackedWidget* results_widget_{nullptr};

    QPushButton* sections_button_{nullptr};
    std::unique_ptr<PopupMenu> sections_menu_;

    QPushButton* back_button_ {nullptr};
    std::vector<std::string> id_history_;

    QLabel*  current_section_label_ = nullptr;
    Section* current_section_       = nullptr;
};

}
