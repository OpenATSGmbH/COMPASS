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

#include "ui_test_testable.h"

#include <QIcon>
#include <QPointer>
#include <QWidget>

#include <string>
#include <vector>

class DeckButton;

class QPushButton;
class QStackedWidget;
class QToolButton;
class QVBoxLayout;

/**
 * View Deck: an icon bar with one button per view, and the view area that shows the selected view.
 * This widget is the icon bar, the host places the view area (viewArea()).
 *
 * The deck does not collapse itself. It asks for it (collapseRequestedSignal(), openRequestedSignal())
 * and the host that owns the window layout answers with setCollapsedSlot(). In a window without a host
 * for these requests the deck stays open.
 */
class ViewDeck : public QWidget, public ui_test::UITestable
{
    Q_OBJECT
public:
    ViewDeck(QWidget* parent = nullptr);
    virtual ~ViewDeck();

    QWidget* viewArea() const;

    /**
     * Adds a view after the existing ones. A view leaves the deck when its widget is deleted.
     * @param view_widget   Widget shown in the view area.
     * @param instance_name Name for the object names of the buttons.
     * @param name          Full view name, shown as tooltip.
     * @param label         Short name under the icon.
     * @param tool_tip_info Further tooltip lines, can be empty.
     */
    void addView(QWidget* view_widget,
                 const std::string& instance_name,
                 const QString& name,
                 const QString& label,
                 const QIcon& icon,
                 const QString& tool_tip_info);

    void setViewEnabled(QWidget* view_widget, bool enabled);

    /// selects the view and opens the deck if it is collapsed
    void showView(QWidget* view_widget);

    /// the selected view, also while the deck is collapsed
    QWidget* currentView() const;

    void setAddRemoveEnabled(bool enabled);

    bool collapsed() const { return collapsed_; }

    /// shows the view with the given name, e.g. 'uiset mainwindow.view_deck "Table View 0"'
    bool uiSet(const QString& str) override;

public slots:
    void setCollapsedSlot(bool collapsed);

signals:
    void addViewRequestedSignal();
    void closeViewRequestedSignal(QWidget* view_widget);
    void collapseRequestedSignal();
    void openRequestedSignal();

private:
    struct Entry
    {
        QWidget*     widget       = nullptr;
        DeckButton*  button       = nullptr;
        QToolButton* close_button = nullptr;
        QString      name;
        bool         enabled      = true;
    };

    int indexOf(QWidget* view_widget) const;

    void setCurrentView(QWidget* view_widget);
    void buttonPressed(QWidget* view_widget);
    void viewRemoved(int index);
    void updateButtons();
    void updateButtonSizes();
    void updateSizePolicies();

    std::vector<Entry> entries_; // same order as the pages of view_area_

    QPointer<QStackedWidget> view_area_;
    QVBoxLayout*             button_layout_ = nullptr;
    QPushButton*             add_button_    = nullptr;

    bool add_remove_enabled_ = true;
    bool collapsed_          = false;
};
