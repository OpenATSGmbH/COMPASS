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

#include "compass_app_export.h"

#include "ui_test_testable.h"
#include "decklayout.h"

#include <vector>

#include <QWidget>

#include <memory>
#include <set>

class DeckButton;
class ToolBoxWidget;
class PopupMenu;
class ViewDeck;

class QPushButton;
class QVBoxLayout;
class QStackedWidget;
class QLabel;
class QMenu;
class QToolBar;
class QFrame;
class QSplitter;

/**
 * Flight Deck: tool button column and the splitter with the tool panel and the view area.
 *
 * It also decides which of the two decks of the main window are open (deck::Layout): the Flight Deck with
 * its tool panel, and the View Deck with the view area. The View Deck asks for a change and follows
 * viewDeckCollapsedSignal(), see setViewDeck().
 */
class COMPASS_APP_API ToolBox : public QWidget, public ui_test::UITestable
{
    Q_OBJECT
public:
    ToolBox(QWidget* parent = nullptr);
    virtual ~ToolBox();

    /// selects the tool with the given name, e.g. 'uiset mainwindow.toolbox "Filters"'
    bool uiSet(const QString& str) override { return selectTool(str.toStdString()); }

    void setViewDeck(ViewDeck* view_deck);

    unsigned int addTool(ToolBoxWidget* tool);
    size_t numTools() const;

    void disableTools(std::set<unsigned int> indexes);

    /// own button size from the tool labels, the set one once set
    QSize buttonSize() const;
    /// sets the fixed size of all tool buttons
    void setButtonSize(const QSize& size);

    void adjustSizings();

    void selectTool(size_t idx);
    bool selectTool(const std::string& name);

    void loadingStarted();
    void loadingDone();

    static const int ToolNameFontSize;

signals:
    void toolChanged();
    void viewDeckCollapsedSignal(bool collapsed);

protected:
    virtual void resizeEvent(QResizeEvent* event) override;

private slots:
    void collapseViewDeckSlot();
    void openViewDeckSlot();

private:
    struct Tool
    {
        int            idx    = -1;
        ToolBoxWidget* widget = nullptr;
        DeckButton*    button = nullptr;
        bool           hidden = false;
        int            width_percent = 0;
    };

    void createUI();
    void updateMenu();
    void updateToolBar();
    void updateButtons();
    void toolActivated(int idx);

    int  activeTool() const;
    bool viewDeckOpen() const;
    void setState(deck::Layout layout, int tool_idx);

    void setWidthPercent(int percent);
    void applyWidthPercent();
    void handleDragged(int panel_width);
    void shrink();
    void grow();

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
    QToolBar*       tool_bar_default_     = nullptr;
    QToolBar*       tool_bar_             = nullptr;
    QSplitter*      splitter_             = nullptr;
    QWidget*        main_widget_          = nullptr;
    QVBoxLayout*    main_layout_          = nullptr;
    QWidget*        main_content_widget_  = nullptr;

    QSize button_size_; // fixed size of the tool buttons, invalid until set

    std::unique_ptr<PopupMenu> config_menu_;

    // no tool is selected at startup, so only the View Deck is open
    deck::Layout deck_layout_ = deck::Layout::ViewDeckOnly;
    int          tool_idx_    = -1; // selected tool, kept while the Flight Deck is collapsed
};
