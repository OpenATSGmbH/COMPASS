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

#include "variableviewannotationwidget.h"
#include "variableview.h"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QLabel>

/**
*/
VariableViewAnnotationWidget::VariableViewAnnotationWidget(const VariableView* view, QWidget* parent)
:   QWidget(parent)
,   view_  (view  )
{
    traced_assert(view_);

    createUI();
    updateContent();
}

/**
*/
VariableViewAnnotationWidget::~VariableViewAnnotationWidget() = default;

/**
*/
void VariableViewAnnotationWidget::createUI()
{
    QFormLayout* layout = new QFormLayout;

    group_label_ = new QLabel;
    group_combo_ = new QComboBox;

    QHBoxLayout* layout_h = new QHBoxLayout;
    layout_h->setSpacing(0);
    layout_h->setMargin(0);

    layout_h->addWidget(group_label_);
    layout_h->addWidget(group_combo_);

    layout->addRow("Plot Group:", layout_h);

    anno_combo_ = new QComboBox;
    layout->addRow("Plot:", anno_combo_);

    connect(group_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &VariableViewAnnotationWidget::groupChanged);
    connect(anno_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &VariableViewAnnotationWidget::annotationChanged);

    setLayout(layout);
}

/**
*/
void VariableViewAnnotationWidget::updateContent()
{
    int group_idx = view_->currentAnnotationGroupIdx();
    int anno_idx  = view_->currentAnnotationIdx();

    //setup annotation groups
    updateGroups(group_idx);

    //setup annotations
    updateAnnotations(anno_idx);
}

/**
*/
void VariableViewAnnotationWidget::groupChanged()
{
    updateAnnotations();

    emit currentAnnotationChanged();
}

/**
*/
void VariableViewAnnotationWidget::updateGroups(int current_idx)
{
    const auto& annotations = view_->annotations();

    size_t num_anno_groups = annotations.size();

    group_combo_->blockSignals(true);
    group_combo_->clear();

    group_combo_->setVisible(num_anno_groups >  1);
    group_label_->setVisible(num_anno_groups <= 1);

    if (num_anno_groups == 0)
        group_label_->setText("-");
    else if (num_anno_groups == 1)
        group_label_->setText(QString::fromStdString(annotations[ 0 ].name));
    else
        group_label_->setText("");

    for (const auto& group : annotations)
        group_combo_->addItem(QString::fromStdString(group.name));
    
    if (group_combo_->count() > 0)
        group_combo_->setCurrentIndex(current_idx >= 0 ? current_idx : 0);

    group_combo_->blockSignals(false);
}

/**
*/
void VariableViewAnnotationWidget::updateAnnotations(int current_idx)
{
    anno_combo_->blockSignals(true);
    anno_combo_->clear();

    int group_idx = group_combo_->currentIndex();

    if (group_idx >= 0)
    {
        const auto& annotations = view_->annotations();
        const auto& group       = annotations[ group_idx ];

        for (const auto& a : group.annotations)
            anno_combo_->addItem(QString::fromStdString(a.metadata.fullTitle()));

        if (anno_combo_->count() > 0)
            anno_combo_->setCurrentIndex(current_idx >= 0 ? current_idx : 0);
    }

    anno_combo_->blockSignals(false);
}

/**
*/
void VariableViewAnnotationWidget::annotationChanged()
{
    emit currentAnnotationChanged();
}

/**
*/
bool VariableViewAnnotationWidget::hasCurrentAnnotation() const
{
    return (currentGroupIdx() >= 0 && currentAnnotationIdx() >= 0);
}

/**
*/
int VariableViewAnnotationWidget::currentGroupIdx() const
{
    return group_combo_->currentIndex();
}

/**
*/
int VariableViewAnnotationWidget::currentAnnotationIdx() const
{
    return anno_combo_->currentIndex();
}
