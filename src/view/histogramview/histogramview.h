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
#include "histogram_raw.h"

class HistogramViewWidget;
class HistogramViewDataSource;
class HistogramViewDataWidget;

namespace dbContent 
{
    class Variable;
    class MetaVariable;
}

/**
*/
class HistogramView : public VariableView
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

    /// @brief Constructor
    HistogramView(const std::string& class_id, 
                  const std::string& instance_id, 
                  ViewContainer* w,
                  ViewManager& view_manager);
    /// @brief Destructor
    virtual ~HistogramView() override;

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id) override;

    /// @brief Returns the used data source
    HistogramViewDataSource* getDataSource()
    {
        traced_assert(data_source_);
        return data_source_;
    }

    virtual void accept(LatexVisitor& v) override;

    virtual bool canShowAnnotations() const override final { return true; }
    virtual std::set<std::string> acceptedAnnotationFeatureTypes() const override;

    bool useLogScale() const;
    void useLogScale(bool value, bool notify_changes);

    static const std::string ParamUseLogScale;

signals:
    void showOnlySelectedSignal(bool value);
    void usePresentationSignal(bool value);
    void showAssociationsSignal(bool value);

protected:
    friend class LatexVisitor;

    virtual void checkSubConfigurables() override;
    virtual void updateSelection() override;

    virtual bool init_impl() override;

    virtual bool refreshScreenOnNeededReload() const override { return true; }

    virtual ViewInfos viewInfos_impl() const override;
    virtual void viewInfoJSON_impl(nlohmann::json& info) const override;

    virtual dbContent::VariableSet getBaseSet(const std::string& dbcontent_name) override;

    HistogramViewDataWidget* getDataWidget();
    const HistogramViewDataWidget* getDataWidget() const;

    /// For data display
    HistogramViewWidget* widget_{nullptr};
    /// For data loading
    HistogramViewDataSource* data_source_{nullptr};

    Settings settings_;
};
