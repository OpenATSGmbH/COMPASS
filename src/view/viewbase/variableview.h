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

#include "view.h"
#include "property.h"
#include "plotmetadata.h"

#include <memory>
#include <vector>

class ViewVariable;
class VariableViewDataWidget;
class VariableViewConfigWidget;
class ViewableDataConfig;

/**
*/
class VariableView : public View
{
public:
    struct Annotation
    {
        PlotMetadata   metadata;
        nlohmann::json feature_json;
    };

    struct AnnotationGroup
    {
        std::string             name;
        std::vector<Annotation> annotations;
    };

    VariableView(const std::string& class_id, 
                 const std::string& instance_id,
                 ViewContainer* w,
                 ViewManager& view_manager);
    virtual ~VariableView();

    virtual void loadingDone() override final;

    ViewVariable& variable(int idx);
    const ViewVariable& variable(int idx) const;
    size_t numVariables() const;

    virtual dbContent::VariableSet getSet(const std::string& dbcontent_name) override final;

    virtual bool canShowAnnotations() const { return false; }

    void showVariables();
    bool showsVariables() const;

    void switchVariables(int var0, int var1, bool inform_config_widget);

    void showAnnotation();
    bool showsAnnotation() const;
    void setCurrentAnnotation(int group_idx, int annotation_idx);
    bool hasAnnotations() const;

    const std::vector<AnnotationGroup>& annotations() const;
    int currentAnnotationGroupIdx() const;
    int currentAnnotationIdx() const;
    const Annotation& currentAnnotation() const;
    bool hasCurrentAnnotation() const;

    bool hasViewPoint () { return current_view_point_ != nullptr; }
    const ViewableDataConfig& viewPoint() { traced_assert(hasViewPoint()); return *current_view_point_; }

public slots:
    virtual void unshowViewPointSlot (const ViewableDataConfig* vp) override final;
    virtual void showViewPointSlot (const ViewableDataConfig* vp) override final;

protected:
    friend class ViewVariable;

    ViewVariable& addVariable(const std::string& id,
                              const std::string& display_name,
                              const std::string& var_name,
                              const std::string& default_dbcont,
                              const std::string& default_name,
                              bool show_meta_vars,
                              bool show_empty_vars,
                              bool allow_empty_var,
                              const std::vector<PropertyDataType>& valid_data_types);
    void addVariablesToSet(dbContent::VariableSet& set, 
                           const std::string& dbcontent_name);

    virtual dbContent::VariableSet getBaseSet(const std::string& dbcontent_name) = 0;

    virtual void preVariableChangedEvent(int idx, const std::string& dbcont, const std::string& name) {}
    virtual void postVariableChangedEvent(int idx) {}

    virtual void unshowViewPoint(const ViewableDataConfig* vp) {}
    virtual void showViewPoint(const ViewableDataConfig* vp) {}

    virtual bool refreshScreenOnNeededReload() const override { return true; }

    virtual void viewInfoJSON_impl(nlohmann::json& info) const override;

private:
    VariableViewDataWidget* getDataWidget();
    VariableViewConfigWidget* getConfigWidget();

    void clearAnnotations();
    void scanViewPointForAnnotations();
    void onShowAnnotationChanged(bool update_config_widget);
    void onEvalResultsChanged();

    void showVariables(bool force, bool update_config);
    void showAnnotation(bool force, bool update_config);

    std::vector<std::unique_ptr<ViewVariable>> variables_;

    std::vector<AnnotationGroup> annotations_;
    int                          current_annotation_group_idx_ = -1;
    int                          current_annotation_idx_       = -1;
    bool                         show_annotation_              = false;

    const ViewableDataConfig* current_view_point_ {nullptr};
};
