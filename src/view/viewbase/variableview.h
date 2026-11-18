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

#include "view.h"
#include "property.h"
#include "plotmetadata.h"

#include <memory>
#include <set>
#include <string>
#include <vector>

class ViewVariable;
class VariableViewConfigWidget;
class VariableViewModel;
class ViewableDataConfig;

/**
*/
class COMPASS_VIEW_API VariableView : public View
{
    Q_OBJECT

signals:
    /// Emitted whenever annotations_ is repopulated or cleared. Subscribers
    /// (e.g. data widgets) refresh the annotations subtree of their layer
    /// panel from annotations() / currentAnnotationGroupIdx() /
    /// currentAnnotationIdx(). With a view model (setModel()) the view does this itself.
    void annotationsChangedSignal();

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

    VariableView(nlohmann::json& config, ViewContainer* parent);
    virtual ~VariableView();

    /// The annotations of a view point with the given feature types, in the order the views show them:
    /// groups sorted by the plot group name, inside a group in the order found.
    /// The report figures use the same order (report_figure::selectRenderEntries()).
    static std::vector<AnnotationGroup> scanForAnnotations(const nlohmann::json& view_point,
                                                           const std::set<std::string>& feature_types);

    virtual void loadingDone() override;

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
    ViewableDataConfig& viewPoint() { traced_assert(hasViewPoint()); return *current_view_point_; }

public slots:
    virtual void unshowViewPointSlot (ViewableDataConfig* vp) override final;
    virtual void showViewPointSlot (ViewableDataConfig* vp) override final;

protected:
    /// a stored variable not available in the opened database is set to empty, if the variable may be empty
    virtual void adaptConfigToDatabase() override;

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

    /// The model of a view that is split into model and render widget (NOT owned). The view gives it
    /// the current annotation and keeps the annotations subtree of its layer panel up to date.
    void setModel(VariableViewModel* model);

    virtual void preVariableChangedEvent(int idx, const std::string& dbcont, const std::string& name) {}
    virtual void postVariableChangedEvent(int idx) {}

    virtual void unshowViewPoint(ViewableDataConfig* vp) {}
    virtual void showViewPoint(ViewableDataConfig* vp) {}

    virtual bool refreshScreenOnNeededReload() const override { return true; }

    virtual void viewInfoJSON_impl(nlohmann::json& info) const override;

private:
    VariableViewConfigWidget* getConfigWidget();

    void clearAnnotations();
    void scanViewPointForAnnotations();
    void onShowAnnotationChanged();
    /// the model shows the current annotation, or the variables
    void updateModelAnnotation();
    /// annotations subtree of the model's layer panel from the annotations (if attached and shown)
    void updateAnnotationsTree();
    void onEvalResultsChanged();

    void showVariables(bool force);
    void showAnnotation(bool force);

    std::vector<std::unique_ptr<ViewVariable>> variables_;

    std::vector<AnnotationGroup> annotations_;
    int                          current_annotation_group_idx_ = -1;
    int                          current_annotation_idx_       = -1;
    bool                         show_annotation_              = false;

    ViewableDataConfig* current_view_point_ {nullptr};

    VariableViewModel* model_ {nullptr}; // see setModel()
};
