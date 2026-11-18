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

#include "variableview.h"
#include "scatterplotdisplaydata.h"

#include <memory>

class QColor;
class ViewContainer;
class ScatterPlotViewWidget;
class ScatterPlotViewDataSource;
class ScatterPlotViewModel;

/**
*/
class ScatterPlotView : public VariableView
{
    Q_OBJECT

signals:
    void showOnlySelectedSignal(bool value);
    void usePresentationSignal(bool value);
    void showAssociationsSignal(bool value);

public:
    struct Settings
    {
        bool use_connection_lines {false};
    };

    enum class Variable
    {
        DataVarX = 0,
        DataVarY
    };

    // ScatterPlotView(const std::string& class_name,
    //                 const std::string& instance_name,
    //                 ViewContainer* w,
    //                 ViewManager& view_manager);
    ScatterPlotView(nlohmann::json& config, ViewContainer* parent);
    /// @brief Destructor
    virtual ~ScatterPlotView() override;

    /// @brief Returns the used data source
    ScatterPlotViewDataSource* getDataSource()
    {
        traced_assert(data_source_);
        return data_source_;
    }

    virtual void accept(LatexVisitor& v) override;

    std::string deckNamePrefix() const override;
    QIcon deckIcon() const override;

    virtual bool canShowAnnotations() const override final { return true; }
    virtual std::set<std::string> acceptedAnnotationFeatureTypes() const override;

    bool useConnectionLines();
    void useConnectionLines(bool value, bool redraw = true);

    ScatterPlotViewModel& model() { return *model_; }
    const ScatterPlotViewModel& model() const { return *model_; }

    /// the render options of the render widget, from the settings of the view
    ScatterPlotRenderOptions renderOptions() const;

    virtual void loadingDone() override;

    static const std::string ParamUseConnectionLines;

    /// above this number of loaded values a load turns the connection lines off
    static const int ConnectLinesDataCountMax;

protected:
    friend class LatexVisitor;

    virtual GenerateResult generateSubConfigurable_impl(nlohmann::json& child_json) override;

    virtual void checkSubConfigurables() override;
    virtual void updateSelection_impl() override;

    virtual bool init_impl() override;
    virtual void viewInfoJSON_impl(nlohmann::json& info) const override;

    virtual dbContent::VariableSet getBaseSet(const std::string& dbcontent_name) override;

    virtual void unshowViewPoint(ViewableDataConfig* vp) override;
    virtual void showViewPoint(ViewableDataConfig* vp) override;

    /// the display data can carry its own connection lines value, the setting follows it
    void displayDataChanged();

    /// For data processing, lives longer than the widget
    std::unique_ptr<ScatterPlotViewModel> model_;
    /// For data display
    ScatterPlotViewWidget* widget_{nullptr};
    /// For data loading
    ScatterPlotViewDataSource* data_source_{nullptr};

    Settings settings_;
};
