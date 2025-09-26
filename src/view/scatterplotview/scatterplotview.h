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

class ScatterPlotViewWidget;
class ScatterPlotViewDataSource;
class ScatterPlotViewDataWidget;

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

    /// @brief Constructor
    ScatterPlotView(const std::string& class_id, 
                    const std::string& instance_id, 
                    ViewContainer* w,
                    ViewManager& view_manager);
    /// @brief Destructor
    virtual ~ScatterPlotView() override;

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id) override;

    /// @brief Returns the used data source
    ScatterPlotViewDataSource* getDataSource()
    {
        traced_assert(data_source_);
        return data_source_;
    }

    virtual void accept(LatexVisitor& v) override;

    virtual bool canShowAnnotations() const override final { return true; }
    virtual std::set<std::string> acceptedAnnotationFeatureTypes() const override;

    bool useConnectionLines();
    void useConnectionLines(bool value, bool redraw = true);

    static const std::string ParamUseConnectionLines;

    ScatterPlotViewDataWidget* getDataWidget();

protected:
    friend class LatexVisitor;

    virtual void checkSubConfigurables() override;
    virtual void updateSelection() override;

    virtual bool init_impl() override;
    virtual void viewInfoJSON_impl(nlohmann::json& info) const override;

    virtual dbContent::VariableSet getBaseSet(const std::string& dbcontent_name) override;

    virtual void unshowViewPoint(const ViewableDataConfig* vp) override;
    virtual void showViewPoint(const ViewableDataConfig* vp) override;

    /// For data display
    ScatterPlotViewWidget* widget_{nullptr};
    /// For data loading
    ScatterPlotViewDataSource* data_source_{nullptr};

    Settings settings_;
};
