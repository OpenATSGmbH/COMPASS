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

#include "compass_view_export.h"

#include "variableview.h"
#include "histogramdisplaydata.h"

#include <memory>

class HistogramViewWidget;
class HistogramViewDataSource;
class HistogramViewModel;

namespace dbContent 
{
    class Variable;
    class MetaVariable;
}

/**
*/
class COMPASS_VIEW_API HistogramView : public VariableView
{
    Q_OBJECT
public:
    struct Settings
    {
        Settings();

        bool use_log_scale;
    };

    enum class Variable
    {
        DataVar = 0,
    };

    HistogramView(nlohmann::json& config, ViewContainer* parent);
    /// @brief Destructor
    virtual ~HistogramView() override;

    /// @brief Returns the used data source
    HistogramViewDataSource* getDataSource()
    {
        traced_assert(data_source_);
        return data_source_;
    }

    virtual void accept(LatexVisitor& v) override;

    std::string deckNamePrefix() const override;
    QIcon deckIcon() const override;

    virtual bool canShowAnnotations() const override final { return true; }
    virtual std::set<std::string> acceptedAnnotationFeatureTypes() const override;

    bool useLogScale() const;
    void useLogScale(bool value, bool notify_changes);

    HistogramViewModel& model() { return *model_; }
    const HistogramViewModel& model() const { return *model_; }

    /// the render options of the render widget, from the settings of the view
    HistogramRenderOptions renderOptions() const;

    static const std::string ParamUseLogScale;

signals:
    void showOnlySelectedSignal(bool value);
    void usePresentationSignal(bool value);
    void showAssociationsSignal(bool value);

protected:
    friend class LatexVisitor;

    virtual GenerateResult generateSubConfigurable_impl(nlohmann::json& child_json) override;

    virtual void checkSubConfigurables() override;
    virtual void updateSelection_impl() override;

    virtual bool init_impl() override;

    virtual bool refreshScreenOnNeededReload() const override { return true; }

    virtual ViewInfos viewInfos_impl() const override;
    virtual void viewInfoJSON_impl(nlohmann::json& info) const override;

    virtual dbContent::VariableSet getBaseSet(const std::string& dbcontent_name) override;

    /// the display data can carry its own log scale, the setting follows it
    void displayDataChanged();

    /// For data processing, lives longer than the widget
    std::unique_ptr<HistogramViewModel> model_;
    /// For data display
    HistogramViewWidget* widget_{nullptr};
    /// For data loading
    HistogramViewDataSource* data_source_{nullptr};

    Settings settings_;
};
