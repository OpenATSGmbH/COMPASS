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

#include "toolboxdefs.h"
#include "ui_test_testable.h"

#include <vector>

#include <QWidget>
#include <QToolButton>
#include <QMouseEvent>

#include <boost/optional.hpp>

#include <memory>
#include <set>

class ToolBoxWidget;
class PopupMenu;

class QPushButton;
class QVBoxLayout;
class QStackedWidget;
class QLabel;
class QMenu;
class QToolBar;
class QFrame;

/**
 */
class ToolButton : public QToolButton
{
    Q_OBJECT
public:
    ToolButton(QWidget* parent = nullptr) : QToolButton(parent) {}
    virtual ~ToolButton() = default;

signals:
    void rightClicked();

protected:
    virtual void mousePressEvent(QMouseEvent *e) override
    {
        if (e->button() == Qt::RightButton)
        {
            emit rightClicked();
            return;
        }

        QToolButton::mousePressEvent(e);
    }
};
 
/**
 */
class ToolBox : public QWidget, public ui_test::UITestable
{
    Q_OBJECT
public:
    ToolBox(QWidget* parent = nullptr);
    virtual ~ToolBox();

    /// selects the tool with the given name, e.g. 'uiset mainwindow.toolbox "Filters"'
    bool uiSet(const QString& str) override { return selectTool(str.toStdString()); }

    void setMainContent(QWidget* content);

    unsigned int addTool(ToolBoxWidget* tool);
    size_t numTools() const;

    void disableTools(std::set<unsigned int> indexes);

    void finalize();
    void adjustSizings();

    void selectTool(size_t idx);
    bool selectTool(const std::string& name);

    bool isExpanded() const;

    boost::optional<toolbox::ScreenRatio> currentScreenRatio() const;

    void loadingStarted();
    void loadingDone();

    static const int   ToolIconSize;
    static const int   ToolNameFontSize;
    static const int   ToolLabelFontSize;
    static const float ExpansionFactor;

signals:
    void toolChanged();

protected:
    virtual void resizeEvent(QResizeEvent* event) override;

private:
    struct Tool
    {
        int            idx    = -1;
        ToolBoxWidget* widget = nullptr;
        ToolButton*    button = nullptr;
        bool           hidden = false;
    };

    void createUI();
    void updateMenu();
    void updateToolBar();
    void updateButtons();
    void toolActivated(int idx);

    void screenRatioChanged(toolbox::ScreenRatio screen_ratio);
    void shrink();
    void grow();
    void toggleExpansion();

    std::vector<Tool> tools_;

    QVBoxLayout*    icon_layout_          = nullptr;
    QStackedWidget* widget_stack_         = nullptr;
    QWidget*        panel_widget_         = nullptr;
    QVBoxLayout*    panel_layout_         = nullptr;
    QFrame *        panel_content_widget_ = nullptr; 
    QLabel*         tool_name_label_      = nullptr;
    QPushButton*    config_button_        = nullptr;
    QAction*        shrink_action_        = nullptr;
    QAction*        grow_action_          = nullptr;
    QAction*        expand_action_        = nullptr;
    QToolBar*       tool_bar_default_     = nullptr;
    QToolBar*       tool_bar_             = nullptr;
    QWidget*        main_widget_          = nullptr;
    QVBoxLayout*    main_layout_          = nullptr;
    QWidget*        main_content_widget_  = nullptr;

    std::unique_ptr<PopupMenu> config_menu_;

    int  active_tool_idx_ = -1;
    bool expanded_        = false;
};
