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
#include <QMenu>
#include <QObject>

#include "configurable.h"
#include "subconfigurablefactory.h"

class MainLoadWidget;
class View;
class ViewDeck;
class ViewManager;
class QHBoxLayout;
class QPushButton;
class QWidget;

class ViewManager;

class ViewContainer : public QObject, public Configurable
{
    Q_OBJECT

  public slots:
    void showAddViewMenuSlot();
    // void saveViewTemplate ();
    void closeViewSlot(QWidget* view_widget);
    void addNewViewSlot();

  public:
    // views by class name, display name shown in the add view menus; further views are added by their libraries
    DECLARE_SUBCONFIGURABLE_FACTORY(viewFactory, View, ViewContainer, None)

    ViewContainer(nlohmann::json& config, ViewManager& view_manager,
                  Configurable* parent, ViewDeck* view_deck, int window_cnt);
    virtual ~ViewContainer();

    const std::vector<std::unique_ptr<View>>& getViews() const;

    ViewManager& viewManager() { return view_manager_; }

    virtual std::string getWindowName();
    //static unsigned int getViewCount() { return view_count_; }

    void addView(const std::string& class_name);
    void enableView(QWidget* widget, bool value);
    /// selects the view in the View Deck and opens the deck if it is collapsed
    void showView(QWidget* widget);

    /// View selected in the View Deck of this container (also while the deck
    /// is collapsed), nullptr if the container is empty.
    View* currentView() const;

    /// true if the View Deck shows only its icon bar, so no view is visible
    bool viewDeckCollapsed() const;

    void resetToStartupConfiguration();

    /// creation sequence number, a higher number means created later
    std::uint64_t created() const { return creation_index_; }

  protected:
    ViewManager& view_manager_;

    ViewDeck* view_deck_{nullptr};

    int window_cnt_{0};
    bool disable_add_remove_views_ {false};

    std::vector<std::unique_ptr<View>> views_;

    std::uint64_t creation_index_ = 0;

    //static unsigned int view_count_;

    virtual void checkSubConfigurables();
    virtual void childCreated(Configurable* child) override;
    void addView(View* view);
};
