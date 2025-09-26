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
#include "tableviewdatasource.h"

class TableViewWidget;
class TableViewDataWidget;

class TableView : public View
{
    Q_OBJECT
public slots:
    virtual void unshowViewPointSlot (const ViewableDataConfig* vp) override;
    virtual void showViewPointSlot (const ViewableDataConfig* vp) override;

// signals:
//     void showOnlySelectedSignal(bool value);
//     void usePresentationSignal(bool value);

public:
    struct Settings
    {
        Settings();

        bool show_only_selected_{false};
        bool use_presentation_{true};
        bool ignore_non_target_reports_{true};
    };

    TableView(const std::string& class_id, const std::string& instance_id, ViewContainer* w,
                ViewManager& view_manager);
    virtual ~TableView() override;

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id) override;

    /// @brief Returns the used data source
    TableViewDataSource* getDataSource()
    {
        traced_assert(data_source_);
        return data_source_;
    }
    TableViewDataWidget* getDataWidget();

    virtual dbContent::VariableSet getSet(const std::string& dbcontent_name) override;

    bool usePresentation() const;
    void usePresentation(bool value);

    bool showOnlySelected() const;
    void showOnlySelected(bool value);

    bool ignoreNonTargetReports() const;
    void ignoreNonTargetReports(bool value);

    virtual void accept(LatexVisitor& v) override;

    const Settings& settings() const;

protected:
    friend class LatexVisitor;

    virtual void checkSubConfigurables() override;
    virtual void updateSelection() override;

    virtual bool init_impl() override;

    virtual void onConfigurationChanged_impl(const std::vector<std::string>& changed_params) override;

    virtual void viewInfoJSON_impl(nlohmann::json& info) const override;


    TableViewWidget* widget_{nullptr};
    TableViewDataSource* data_source_{nullptr};

    Settings settings_;
};
