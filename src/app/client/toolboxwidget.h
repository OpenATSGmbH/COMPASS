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

#include <string>

#include <QWidget>
#include <QIcon>

#include <boost/optional.hpp>

class QMenu;
class QToolBar;

/**
 * Base class for widgets which are part of the toolbox.
 */
class ToolBoxWidget : public QWidget
{
    Q_OBJECT

signals:
    void iconChangedSignal();
    void toolsChangedSignal();

public:
    ToolBoxWidget(QWidget* parent = nullptr);
    virtual ~ToolBoxWidget();

    void setScreenRatio(toolbox::ScreenRatio screen_ratio) { screen_ratio_custom_ = screen_ratio; }
    toolbox::ScreenRatio screenRatio() const { return screen_ratio_custom_.has_value() ? screen_ratio_custom_.value() : defaultScreenRatio(); }

    /// returns the tool's button icon
    virtual QIcon toolIcon() const = 0;

    /// returns the tool's name
    virtual std::string toolName() const = 0;

    /// return information about the tool
    virtual std::string toolInfo() const = 0;

    /// returns the the tool's button label devided into spearate rows (e.g. { "Data", "Source" })
    virtual std::vector<std::string> toolLabels() const = 0;

    /// returns the tool's default screen ratio
    virtual toolbox::ScreenRatio defaultScreenRatio() const { return ScreenRatioDefault; };

    virtual bool checkable() const { return false; }

    /// adds the tool's config actions/menus to the given menu
    virtual void addToConfigMenu(QMenu* menu) {};

    /// adds tool actions to the given toolbar
    virtual void addToToolBar(QToolBar* tool_bar) {};

    /// reacts on right clicks on the tools button
    virtual void rightClicked() {}

    /// react on data loading
    virtual void loadingStarted() {}
    virtual void loadingDone() {}

    static const toolbox::ScreenRatio ScreenRatioDefault;
    
private:
    boost::optional<toolbox::ScreenRatio> screen_ratio_custom_;
};

/**
 * ToolBoxWidget which wraps a widget not derived from ToolBoxWidget. 
 * For convenience - missing information has to be passed in the constructor.
 */
class WrappedToolBoxWidget : public ToolBoxWidget
{
public:
    WrappedToolBoxWidget(QWidget* w, 
                         const std::string& name,
                         const std::string& info,
                         const std::vector<std::string>& labels,
                         const QIcon& icon,
                         bool checkable,
                         toolbox::ScreenRatio screen_ratio_default = ToolBoxWidget::ScreenRatioDefault,
                         const std::function<void(QMenu*)>& addToConfigMenu_cb = std::function<void(QMenu*)>(),
                         const std::function<void(QToolBar*)>& addToToolBar_cb = std::function<void(QToolBar*)>(),
                         QWidget* parent = nullptr);
    virtual ~WrappedToolBoxWidget();

    QIcon toolIcon() const override final;
    std::string toolName() const override final;
    std::string toolInfo() const override final;
    std::vector<std::string> toolLabels() const override final;
    toolbox::ScreenRatio defaultScreenRatio() const override final;
    bool checkable() const override final;
    void addToConfigMenu(QMenu* menu) override final;
    void addToToolBar(QToolBar* tool_bar) override final;

private:
    std::string                    name_;
    std::string                    info_;
    std::vector<std::string>       labels_;
    QIcon                          icon_;
    bool                           checkable_;
    toolbox::ScreenRatio           screen_ratio_default_;
    std::function<void(QMenu*)>    addToConfigMenu_cb_;
    std::function<void(QToolBar*)> addToToolBar_cb_;
};
