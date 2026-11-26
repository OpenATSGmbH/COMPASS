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

#include "layertreeitem.h"
#include "variableview.h"

#include <QIcon>

#include <map>
#include <optional>
#include <string>
#include <vector>

/**
 * "Annotations" root item placed as a sibling to the DBContent root in a
 * LayerPanelWidget. Mirrors the Geographic View's Annotations item in name
 * and icon (compass.png).
 *
 * Children are AnnotationGroupItems (one per PlotMetadata::plot_group_) each
 * holding a flat list of AnnotationLeafItems. The variable-view annotation
 * model is consumer-side flat: VariableView::scanViewPointForAnnotations
 * collapses any nested feature hierarchy into 2 levels keyed by plot_group_.
 *
 * Lifecycle: data widgets call updateFrom() in response to
 * VariableView::annotationsChangedSignal. The subtree is updated in place:
 * recurring groups and annotations (by name / title) are kept, so expansion
 * and selection survive natively.
 */
class AnnotationsRootItem : public LayerTreeItem
{
public:
    AnnotationsRootItem();
    ~AnnotationsRootItem() override = default;

    QVariant icon() const override;

    /// Updates the annotation subtree in place from the supplied groups. The leaf at
    /// (current_group_idx, current_anno_idx) is the checked one (radio), every other
    /// leaf unchecked. Empty groups -> empty subtree. Annotations of an unnamed group
    /// sit directly below the root. `view` is passed through to each leaf so radio-style
    /// activation can call setCurrentAnnotation / showAnnotation.
    void updateFrom(const std::vector<VariableView::AnnotationGroup>& groups,
                    int           current_group_idx,
                    int           current_anno_idx,
                    VariableView* view);

    std::optional<std::vector<std::string>> childKeys() const override;
    std::unique_ptr<LayerTreeItem> createChild(const std::string& key) override;
    bool updateChild(LayerTreeItem& child) override;

    // used by the group items during updateFrom(); parent key empty => the root's own leaves

    /// update keys of the leaves below the parent, nullopt outside updateFrom()
    std::optional<std::vector<std::string>> leafKeys(const std::string& parent_key) const;
    std::unique_ptr<LayerTreeItem> createLeaf(const std::string& parent_key, const std::string& key) const;
    /// indices and radio state of a kept leaf, true if its checked state changed
    bool updateLeaf(const std::string& parent_key, LayerTreeItem& child) const;

private:
    struct LeafSpec
    {
        std::string key;
        std::string title;
        int         group_idx      = -1;
        int         annotation_idx = -1;
    };

    struct Layout
    {
        std::vector<std::string>                     root_keys;   // groups and own leaves in display order
        std::map<std::string, std::string>           group_names; // group key -> name
        std::map<std::string, std::vector<LeafSpec>> leaves;      // parent key -> leaves in display order
    };

    const LeafSpec* findLeaf(const std::string& parent_key, const std::string& key) const;
    bool isCurrent(const LeafSpec& spec) const;

    QIcon icon_;

    std::optional<Layout> layout_;                 // during updateFrom() only
    int                   current_group_idx_ = -1;
    int                   current_anno_idx_  = -1;
    VariableView*         view_              = nullptr;
};
