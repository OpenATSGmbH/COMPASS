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

#include "variableviewconfigwidget.h"
#include "variableview.h"
#include "viewvariable.h"

#include "variableviewannotationwidget.h"

#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variableselectionwidget.h"

#include "logger.h"
#include "variable.h"
#include "metavariable.h"
#include "ui_test_common.h"

#include <QVBoxLayout>
#include <QLabel>
#include <QTabWidget>
#include <QRadioButton>
#include <QToolButton>
#include <QPainter>

/**
*/
VariableViewConfigWidget::VariableViewConfigWidget(ViewWidget* view_widget, 
                                                   VariableView* view,
                                                   QWidget* parent)
:   TabStyleViewConfigWidget(view_widget, parent)
,   var_view_(view)
{
    traced_assert(var_view_);

    auto addVariableUI = [ & ] (QVBoxLayout* layout, QVBoxLayout* switch_layout, int idx)
    {
        const auto& var = var_view_->variable(idx);

        std::string label       = var.settings().display_name + " Variable";
        std::string object_name = "variable_selection_" + var.settings().var_name;

        layout->addWidget(new QLabel(QString::fromStdString(label)));

        auto sel_widget = new dbContent::VariableSelectionWidget();
        sel_widget->setObjectName(QString::fromStdString(object_name));

        var_view_->variable(idx).configureWidget(*sel_widget);

        var_selection_widgets_.push_back(sel_widget);
        updateSelectedVariables(idx);

        connect(sel_widget, &dbContent::VariableSelectionWidget::selectionChanged, 
            [ this, idx ] () { this->selectedVariableChangedSlot(idx); } );

        layout->addWidget(sel_widget);

        if (idx > 0)
        {
            int idx0 = idx - 1;
            int idx1 = idx;

            const auto& var0 = var_view_->variable(idx0);

            auto var_w = new QWidget;
            auto layout = new QHBoxLayout;
            layout->setContentsMargins(0, 0, 0, 0);
            var_w->setLayout(layout);

            QImage icon_img_upper(Utils::Files::getIconFilepath("collapse.png").c_str());
            QImage icon_img_lower = icon_img_upper.transformed(QTransform().rotate(180));
            QIcon icon;
            if (!icon_img_upper.isNull())
            {
                int w = icon_img_upper.width();
                int h = icon_img_upper.height();
                int h_half  = h / 2;
                int h_arrow = h_half * 0.7;
                int w_offs  = (w - h_arrow) / 2;
                int h_offs  = (h_half - h_arrow) / 2;

                auto icon_img_upper_scaled = icon_img_upper.scaled(h_arrow, h_arrow);
                auto icon_img_lower_scaled = icon_img_lower.scaled(h_arrow, h_arrow);

                QImage img(w, h_half * 2, icon_img_upper.format());
                QPainter p(&img);
                p.drawImage(QPoint(w_offs, h_offs), icon_img_upper_scaled);
                p.drawImage(QPoint(w_offs, h_half + h_offs), icon_img_lower_scaled);

                icon = QIcon(QPixmap::fromImage(img));
            }

            auto var_switch = new QToolButton;
            var_switch->setIcon(icon);
            var_switch->setToolTip(QString::fromStdString("Switch " + var0.settings().display_name + " and " + var.settings().display_name));
            var_switch->setVisible(false);

            var_w->setFixedSize(var_switch->sizeHint());
            layout->addWidget(var_switch);

            auto switch_cb = [ = ] () { this->switchVariables(idx0, idx1); };

            connect(var_switch, &QToolButton::pressed, switch_cb);

            var_switches_.push_back(var_switch);

            switch_layout->addWidget(var_w);
        }

        switch_layout->addSpacerItem(new QSpacerItem(1, 1, QSizePolicy::Fixed, QSizePolicy::Minimum));
        //switch_layout->addStretch(1);
    };

    bool show_annotation = var_view_->showsAnnotation() && 
                           var_view_->hasCurrentAnnotation();

    QWidget*     cfg_widget = new QWidget();
    QVBoxLayout* cfg_layout = new QVBoxLayout();

    // variables
    {
        show_variables_box_ = new QRadioButton("Show Variable Data");
        show_variables_box_->setChecked(!show_annotation);
        UI_TEST_OBJ_NAME(show_variables_box_, show_variables_box_->text())

        connect(show_variables_box_, &QRadioButton::toggled, this, &VariableViewConfigWidget::dataSourceToggled);

        cfg_layout->addWidget(show_variables_box_);

        variables_widget_ = new QWidget;

        QHBoxLayout* var_layout_outer = new QHBoxLayout;
        variables_widget_->setLayout(var_layout_outer);

        QVBoxLayout* var_layout = new QVBoxLayout;
        var_layout_outer->addLayout(var_layout);

        QVBoxLayout* switch_layout = new QVBoxLayout;
        var_layout_outer->addLayout(switch_layout);

        for (size_t i = 0; i < var_view_->numVariables(); ++i)
            addVariableUI(var_layout, switch_layout, i);

        cfg_layout->addWidget(variables_widget_);
    }

    //eval results
    {
        show_annotations_box_ = new QRadioButton("Show Annotations");
        show_annotations_box_->setChecked(!show_annotation);
        UI_TEST_OBJ_NAME(show_annotations_box_, show_annotations_box_->text())

        connect(show_annotations_box_, &QRadioButton::toggled, this, &VariableViewConfigWidget::dataSourceToggled);

        cfg_layout->addWidget(show_annotations_box_);

        annotation_widget_ = new VariableViewAnnotationWidget(var_view_);

        connect(annotation_widget_, &VariableViewAnnotationWidget::currentAnnotationChanged, this, &VariableViewConfigWidget::annotationChanged);

        cfg_layout->addWidget(annotation_widget_);
    }

    //deactivate annotations related ui if not supported by view
    show_variables_box_->setVisible(var_view_->canShowAnnotations());
    show_annotations_box_->setVisible(var_view_->canShowAnnotations());
    annotation_widget_->setVisible(var_view_->canShowAnnotations());

    config_layout_ = new QVBoxLayout;

    cfg_layout->addLayout(config_layout_);
    cfg_layout->addStretch();

    cfg_widget->setLayout(cfg_layout);

    getTabWidget()->addTab(cfg_widget, "Config");

    //update ui
    updateConfig();
}

/**
*/
VariableViewConfigWidget::~VariableViewConfigWidget() = default;

/**
*/
void VariableViewConfigWidget::selectedVariableChangedSlot(int idx)
{
    loginf << "idx = " << idx;

    auto selection = var_selection_widgets_.at(idx);
    traced_assert(selection);

    var_view_->variable(idx).setVariable(*selection, true);

    variableChangedEvent(idx);
}

/**
*/
void VariableViewConfigWidget::configChanged()
{
    updateSelectedVariables();

    //invoke derived
    configChanged_impl();
}

/**
*/
void VariableViewConfigWidget::updateSelectedVariables()
{
    size_t n = var_view_->numVariables();
    
    for (size_t i = 0; i < n; ++i)
        updateSelectedVariables(i);
}

/**
*/
void VariableViewConfigWidget::updateSelectedVariables(size_t idx)
{
    auto selection = var_selection_widgets_.at(idx);
    traced_assert(selection);

    var_view_->variable(idx).updateWidget(*selection);

    variableChangedEvent(idx);
}

/**
*/
const dbContent::VariableSelectionWidget* VariableViewConfigWidget::variableSelection(size_t idx) const
{
    return var_selection_widgets_.at(idx);
}

/**
*/
bool VariableViewConfigWidget::showsAnnotation() const
{
    return show_annotations_box_->isChecked();
}

/**
*/
void VariableViewConfigWidget::viewInfoJSON_impl(nlohmann::json& info) const
{
    //variable related
    for (size_t i = 0; i < var_selection_widgets_.size(); ++i)
    {
        auto w = var_selection_widgets_[ i ];

        const auto& var = var_view_->variable(i);

        std::string tag = "selected_var_" + var.settings().var_name;

        if (w->hasMetaVariable())
            info[ tag ] = "Meta - " + w->selectedMetaVariable().name();
        else
            info[ tag ] = w->selectedVariable().dbContentName() + " - " + w->selectedVariable().name();
    }

    //eval result related
    info[ "annotations_active"   ] = show_annotations_box_->isChecked();
    info[ "annotation_group_idx" ] = annotation_widget_->currentGroupIdx();
    info[ "annotation_idx"       ] = annotation_widget_->currentAnnotationIdx();
}

/**
 */
void VariableViewConfigWidget::updateConfig()
{
    bool has_annotations  = var_view_->hasAnnotations();
    bool show_annotations = var_view_->showsAnnotation() && has_annotations;

    show_annotations_box_->blockSignals(true);
    show_annotations_box_->setChecked(show_annotations);
    show_annotations_box_->blockSignals(false);

    show_variables_box_->blockSignals(true);
    show_variables_box_->setChecked(!show_annotations);
    show_variables_box_->blockSignals(false);

    traced_assert(annotation_widget_);
    annotation_widget_->updateContent();

    show_annotations_box_->setEnabled(has_annotations);

    annotation_widget_->setEnabled(show_annotations);
    variables_widget_->setEnabled(!show_annotations);
}

/**
 */
void VariableViewConfigWidget::dataSourceToggled()
{
    //modify state in view based on selected radio button
    bool show_anno = show_annotations_box_->isChecked();

    if (show_anno)
        var_view_->showAnnotation();
    else
        var_view_->showVariables();

    //update ui
    updateConfig();

    //invoke derived class
    dataSourceChangedEvent();
}

/**
 */
void VariableViewConfigWidget::annotationChanged()
{
    var_view_->setCurrentAnnotation(annotation_widget_->currentGroupIdx(),
                                    annotation_widget_->currentAnnotationIdx());
}

/**
 */
void VariableViewConfigWidget::showSwitch(int var0, bool ok)
{
    var_switches_.at(var0)->setVisible(ok);
}

/**
 */
void VariableViewConfigWidget::switchVariables(int idx0, int idx1)
{
    var_view_->switchVariables(idx0, idx1, true);
}
