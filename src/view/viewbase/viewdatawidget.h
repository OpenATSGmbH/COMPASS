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

#include <map>
#include <memory>

#include <boost/optional.hpp>

#include <QWidget>

class ViewWidget;
class ViewToolSwitcher;
class Buffer;

namespace dbContent
{
    class Variable;
}

/**
 * Base class for view data widgets, which are held in the data area of the ViewWidget.
 * Used to display data in a view specific way, e.g. as a graph.
 * Derive and reimplement as needed.
 */
class ViewDataWidget : public QWidget, public ViewComponent 
{
    Q_OBJECT
public:
    enum class DrawState
    {
        NotDrawn,    // not drawn yet
        Drawn,       // drawn, but no content
        DrawnContent // drawn with content
    };

    typedef std::map<std::string, std::shared_ptr<Buffer>> BufferData;

    ViewDataWidget(ViewWidget* view_widget, QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~ViewDataWidget() = default;

    void setToolSwitcher(ViewToolSwitcher* tool_switcher);

    void loadingStarted();
    void loadingDone();
    void updateData(const BufferData& buffer_data, bool requires_reset);
    void clearData();
    DrawState redrawData(bool recompute, bool notify = false);
    void liveReload();

    bool hasData() const;
    bool hasAnnotations() const;
    bool hasContent() const;
    bool hasVisibleContent() const;
    bool isDrawn() const;
    bool isContentDrawn() const;

    virtual void isExporting(bool ok) { is_exporting_ = ok; }
    bool isExporting() const { return is_exporting_; }

    virtual bool hasScreenshotContent() const;

    unsigned int loadedDataCount();

    bool isVariableSetLoaded() const;

    virtual void appModeSwitch(AppMode app_mode) {} //reacts on switching the application mode
    virtual void configChanged() {}                 //reacts on configuration changes

    nlohmann::json viewInfoJSON() const override final;

    virtual QImage renderData();

    QColor colorForGroupName(const std::string& group_name); // creates new one of required
    const std::map<std::string, QColor>& dbContentColors() const;

    const boost::optional<size_t>& nullCount() const { return count_null_; }
    const boost::optional<size_t>& nanCount() const { return count_nan_; }

    static const double      MarkerSizePx;
    static const double      MarkerSizeSelectedPx;

    static const std::string Color_CAT001;
    static const std::string Color_CAT010;
    static const std::string Color_CAT020;
    static const std::string Color_CAT021;
    static const std::string Color_CAT048;
    static const std::string Color_RefTraj;
    static const std::string Color_CAT062;

    static const QColor      ColorSelected;

signals:
    void displayChanged();
    void dataLoaded();
    void liveDataLoaded();
    void redrawStarted();
    void redrawDone();
    void updateStarted();
    void updateDone();
    
protected:
    virtual void toolChanged_impl(int tool_id) = 0;        //implements reactions on tool switches
    virtual void loadingStarted_impl() = 0;                //implements behavior at starting a reload
    virtual void loadingDone_impl();                       //implements behavior at finishing a reload
    virtual void updateData_impl(bool requires_reset) = 0; //implements behavior at receiving new data
    virtual void clearData_impl() = 0;                     //implements clearing all view data
    virtual void clearIntermediateRedrawData_impl() = 0;   //implements clearing of any data collected during redraw
    virtual DrawState redrawData_impl(bool recompute) = 0; //implements redrawing the display (and possibly needed computations), and returns the new draw state
    virtual void liveReload_impl() = 0;                    //implements data reload during live running mode
    virtual bool hasAnnotations_impl() const = 0;          //implements checking if the view has any annotations

    virtual void viewInfoJSON_impl(nlohmann::json& info) const {}

    void addNullCount(size_t n);
    void addNanCount(size_t n);

    void endTool();

    void setDrawState(DrawState state) { draw_state_ = state; }

    const BufferData& viewData() const { return data_; }
    BufferData& viewData() { return data_; } //exposed because of selection

    ViewWidget* getWidget() { return view_widget_; }

private:
    friend class ViewLoadStateWidget;

    void clearIntermediateRedrawData();
    void toolChanged(int mode, const QCursor& cursor);

    ViewWidget*       view_widget_   = nullptr;
    ViewToolSwitcher* tool_switcher_ = nullptr;

    BufferData data_;
    DrawState  draw_state_ = DrawState::NotDrawn;

    bool is_exporting_ = false;

    std::map<std::string, QColor> dbc_colors_;

    boost::optional<size_t> count_null_ = 0;
    boost::optional<size_t> count_nan_  = 0;
};
