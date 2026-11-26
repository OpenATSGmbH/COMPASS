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
#include "report/reportdocumentlayout.h"
#include "json.hpp"

#include <boost/optional.hpp>

#include <QWidget>
#include <QTreeView>

class TaskResultsWidget;
class PopupMenu;

class QPushButton;
class QLabel;
class QSplitter;

namespace ResultReport
{

class Section;
class ReportDocumentView;

class ReportWidget : public QWidget
{
    Q_OBJECT

public slots:
    void itemClickedSlot(const QModelIndex& index);
    void itemDblClickedSlot(const QModelIndex& index);
    void contextMenuSlot(const QPoint& pos);

    void stepBackSlot();

    void currentSectionChangedSlot(Section* section);
    void sidebarExpansionChangedSlot();
    void reportAboutToClearSlot();

public:
    ReportWidget(TaskResultsWidget& task_result_widget);
    virtual ~ReportWidget();

    /// whether the section sidebar is shown at the given width of the report widget
    static bool sidebarShownForWidth(bool shown, int width);

    static const int SidebarHideWidth;
    static const int SidebarShowHysteresis;

    void setReport(const std::shared_ptr<Report>& report);
    void clear();
    void releaseDocument();

    void expand();

    void selectID(const std::string& id,
                  bool show_figure,
                  const nlohmann::json& config,
                  bool add_to_history = true);
    void reshowLastId ();

    std::string currentSectionID() const;
    nlohmann::json currentSectionConfig() const;

    void showFigure(const QModelIndex& index);

    static const std::string FieldConfigScrollPosV;
    static const std::string FieldConfigScrollPosH;

protected:
    /**
     * Back history entry: a jump target, or the position the user left with a jump.
     */
    struct HistoryEntry
    {
        std::string    id;     // section id
        nlohmann::json config; // currentSectionConfig() when left (scroll positions, table state), empty = section start
    };

    void expandAllParents (QTreeView* view, QModelIndex index);
    void expandSidebarInitially();

    void resizeEvent(QResizeEvent* event) override;
    void applySidebarShown(bool shown);
    ReportDocumentLayout::SectionSet collapsedSections() const;
    void selectInTrees(const QModelIndex& index);
    QModelIndex modelIndex(Section* section) const;
    void updateBackButton ();
    void updateCurrentSectionLabel();
    void triggerItem (const QModelIndex& index,
                      bool preload_ondemand_contents,
                      bool add_to_history);
    void addToHistory(const std::string& target_id);

    bool configureSection(const nlohmann::json& config);

    TaskResultsWidget& task_result_widget_;

    ResultReport::TreeModel tree_model_;
    QTreeView* tree_view_;
    QTreeView* sidebar_view_ {nullptr};

    QSplitter*          splitter_      {nullptr};
    ReportDocumentView* document_view_ {nullptr};

    QPushButton* sections_button_{nullptr};
    bool         sidebar_shown_ {true};
    std::unique_ptr<PopupMenu> sections_menu_;

    QPushButton* back_button_ {nullptr};
    std::vector<HistoryEntry> id_history_;

    QLabel*  current_section_label_ = nullptr;
    Section* current_section_       = nullptr;

    bool always_show_section_figure_ = true;

    std::shared_ptr<Report> report_;
    QMetaObject::Connection report_clear_connection_;
    bool                    document_outdated_ = false;
};

}
