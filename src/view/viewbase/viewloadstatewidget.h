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

#include "viewcomponent.h"
#include "appmode.h"
#include "json_fwd.hpp"

#include <string>

#include <QWidget>
#include <QColor>

class ViewWidget;

class QLabel;
class QPushButton;

/**
 * Widget keeping/displaying the current data state and handling manual updates like reloading and redrawing.
*/
class ViewLoadStateWidget : public QWidget, public ViewComponent
{
public:
    enum class State
    {
        None = 0,
        NoData,
        Loading,
        Drawing,
        Loaded,
        ReloadRequired,
        RedrawRequired
    };

    enum class InactivityMode
    {
        Show = 0,
        Hide,
        Disable
    };

    ViewLoadStateWidget(ViewWidget* view_widget, QWidget* parent = nullptr);
    virtual ~ViewLoadStateWidget() = default;

    void setState(State state);
    void updateState();
    
    void loadingStarted();
    void loadingDone();
    void redrawStarted();
    void redrawDone();

    void appModeSwitch(AppMode app_mode);

    bool viewUpdateRequired() const;
    bool viewReloadRequired() const;
    bool viewBusy() const;

    nlohmann::json viewInfoJSON() const override final;

    static const int DefaultMargin = 4;

protected:
    virtual void viewInfoJSON_impl(nlohmann::json& info) const {}
    virtual void changeEvent(QEvent* event) override;

private:
    static std::string messageFromState(State state);
    static std::string stringFromState(State state);
    static QColor colorFromState(State state);
    static std::string buttonTextFromState(State state);

    void updateData();

    State           state_          = State::None;
    InactivityMode  inactive_mode_  = InactivityMode::Hide;

    QWidget*        content_widget_ = nullptr;
    QLabel*         status_label_   = nullptr;
    QPushButton*    refresh_button_ = nullptr;

    ViewWidget*     view_widget_    = nullptr;
};
