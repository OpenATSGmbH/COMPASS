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

#include "dbcontent/label/labelgeneratorwidget.h"
#include "dbcontent/label/labelgenerator.h"
#include "dbcontent/label/labeldswidget.h"
#include "compass.h"
#include "dbcontentmanager.h"
#include "logger.h"
#include "files.h"
#include "gui/rangeedit.h"
#include "textfielddoublevalidator.h"

#include <QFormLayout>
#include <QComboBox>
#include <QLabel>
#include <QCheckBox>
#include <QGridLayout>
#include <QLineEdit>
#include <QCheckBox>
#include <QPushButton>
#include <QScrollArea>
#include <QMenu>
#include <QSlider>

using namespace std;
using namespace Utils;

namespace dbContent
{

LabelGeneratorWidget::LabelGeneratorWidget(LabelGenerator& label_generator)
    : label_generator_(label_generator)
{
    QVBoxLayout* main_layout = new QVBoxLayout();

    QHBoxLayout* edit_layout = new QHBoxLayout();
    edit_layout->addStretch();

    QPushButton* ed_edit = new QPushButton();
    ed_edit->setIcon(Files::IconProvider::getIcon("edit.png"));
    ed_edit->setFixedSize(UI_ICON_SIZE);
    ed_edit->setFlat(UI_ICON_BUTTON_FLAT);
    connect(ed_edit, &QPushButton::clicked,
            this, &LabelGeneratorWidget::editSettingsSlot);
    ed_edit->setToolTip("Edit Label Settings");
    edit_layout->addWidget(ed_edit);

    main_layout->addLayout(edit_layout);

    QFormLayout* form_layout1 = new QFormLayout;

    QCheckBox* auto_label_check = new QCheckBox();
    auto_label_check->setChecked(label_generator_.autoLabel());
    connect(auto_label_check, &QCheckBox::clicked,
            this, &LabelGeneratorWidget::autoLabelChangedSlot);
    form_layout1->addRow("Show Automatic Labels", auto_label_check);

    // lod
    QComboBox* lod_box = new QComboBox();
    lod_box->addItems({"Auto", "1", "2", "3"});

    if (label_generator_.autoLOD())
        lod_box->setCurrentText("Auto");
    else
        lod_box->setCurrentText(QString::number(label_generator_.currentLOD()));

    connect(lod_box, &QComboBox::currentTextChanged,
            this, &LabelGeneratorWidget::lodChangedSlot);
    form_layout1->addRow(tr("Level of Detail"), lod_box);

    // opacity
    QSlider* opacity_slider = new QSlider(Qt::Horizontal);
    opacity_slider->setMinimum(0);
    opacity_slider->setMaximum(100);
    opacity_slider->setSingleStep(1);
    opacity_slider->setSliderPosition((1.0 - label_generator_.labelOpacity()) * 100);
    opacity_slider->setMaximumWidth(100);

    connect (opacity_slider, &QSlider::valueChanged,
             this, &LabelGeneratorWidget::opacitySliderChangedSlot);

    form_layout1->addRow(tr("Label Opacity"), opacity_slider);

    // declutter
    QCheckBox* declutter_box = new QCheckBox();
    declutter_box->setChecked(label_generator_.declutterLabels());
    connect(declutter_box, &QCheckBox::clicked,
            this, &LabelGeneratorWidget::declutterLabelsChangedSlot);
    form_layout1->addRow(tr("Declutter Labels"), declutter_box);

    main_layout->addLayout(form_layout1);

    QScrollArea* scroll_area = new QScrollArea();
    scroll_area->setWidgetResizable(true);

    label_ds_widget_ = new LabelDSWidget(label_generator_);
    scroll_area->setWidget(label_ds_widget_);
    scroll_area->setMinimumHeight(300);
    //scroll_area->setMaximumHeight(400);

    main_layout->addWidget(scroll_area);

    unsigned int row=0;

    // filters
    main_layout->addSpacing(10);

    main_layout->addWidget(new QLabel("Label Filters"));

    QScrollArea* scroll_area2 = new QScrollArea();
    scroll_area2->setWidgetResizable(true);

    QFrame* filter_frame = new QFrame();
    scroll_area2->setWidget(filter_frame);

    QGridLayout* filter_layout = new QGridLayout();

    // m3a
    QCheckBox* filter_mode3a_box = new QCheckBox("Mode 3/A Codes");
    filter_mode3a_box->setChecked(label_generator_.filterMode3aActive());
    connect(filter_mode3a_box, &QCheckBox::clicked,
            this, &LabelGeneratorWidget::filterMode3AActiveChangedSlot);
    filter_layout->addWidget(filter_mode3a_box, row, 0);

    QLineEdit* filter_mode3a_edit = new QLineEdit();
    filter_mode3a_edit->setText(label_generator_.filterMode3aValues().c_str());
    connect(filter_mode3a_edit, &QLineEdit::textEdited,
            this, &LabelGeneratorWidget::filterMode3AChangedSlot);
    filter_layout->addWidget(filter_mode3a_edit, row, 1);

    // mc
    {
        const int   SliderSteps =  1000;
        const int   Precision   =  2;
        const float ModeCMin    = -100.0f;
        const float ModeCMax    =  500.0f;

        ++row;
        QCheckBox* filter_modec_min_box = new QCheckBox("Mode C Min [FL]");
        filter_modec_min_box->setChecked(label_generator_.filterModecMinActive());
        connect(filter_modec_min_box, &QCheckBox::clicked,
                this, &LabelGeneratorWidget::filterModeCMinActiveChangedSlot);
        filter_layout->addWidget(filter_modec_min_box, row, 0);

        QLineEdit* filter_modec_min_edit = new QLineEdit();
        auto validator0 = new TextFieldDoubleValidator(ModeCMin, ModeCMax, Precision);
        validator0->setNotation(QDoubleValidator::Notation::StandardNotation);
        filter_modec_min_edit->setValidator(validator0);
        filter_modec_min_edit->setText(QString::number(label_generator_.filterModecMinValue(), 'f', Precision)); 
        connect(filter_modec_min_edit, &QLineEdit::textEdited,
                this, &LabelGeneratorWidget::filterModeCMinChangedSlot);
        filter_layout->addWidget(filter_modec_min_edit, row, 1);

        ++row;
        QCheckBox* filter_modec_max_box = new QCheckBox("Mode C Max [FL]");
        filter_modec_max_box->setChecked(label_generator_.filterModecMaxActive());
        connect(filter_modec_max_box, &QCheckBox::clicked,
                this, &LabelGeneratorWidget::filterModeCMaxActiveChangedSlot);
        filter_layout->addWidget(filter_modec_max_box, row, 0);

        QLineEdit* filter_modec_max_edit = new QLineEdit();
        auto validator1 = new TextFieldDoubleValidator(ModeCMin, ModeCMax, Precision);
        validator1->setNotation(QDoubleValidator::Notation::StandardNotation);
        filter_modec_max_edit->setValidator(validator1);
        filter_modec_max_edit->setText(QString::number(label_generator_.filterModecMaxValue(), 'f', Precision));
        connect(filter_modec_max_edit, &QLineEdit::textEdited,
                this, &LabelGeneratorWidget::filterModeCMaxChangedSlot);
        filter_layout->addWidget(filter_modec_max_edit, row, 1);

        ++row;
        const QString limit0 = QString::number(ModeCMin, 'f', Precision);
        const QString limit1 = QString::number(ModeCMax, 'f', Precision);
        RangeEditFloat* filter_modec_range_edit = new RangeEditFloat(SliderSteps, Precision);
        filter_modec_range_edit->setLimits(limit0, limit1);
        filter_modec_range_edit->connectToFields(filter_modec_min_edit, 
                                                 filter_modec_max_edit);
        filter_layout->addWidget(filter_modec_range_edit, row, 1);
    }

    ++row;
    QCheckBox* filter_modec_null_box = new QCheckBox("Mode C NULL");
    filter_modec_null_box->setChecked(label_generator_.filterModecNullWanted());
    connect(filter_modec_null_box, &QCheckBox::clicked,
            this, &LabelGeneratorWidget::filterModeCNullWantedChangedSlot);
    filter_layout->addWidget(filter_modec_null_box, row, 0);

    // ti
    ++row;
    QCheckBox* filter_ti_box = new QCheckBox("Aircraft Identification");
    filter_ti_box->setChecked(label_generator_.filterTIActive());
    connect(filter_ti_box, &QCheckBox::clicked,
            this, &LabelGeneratorWidget::filterTIActiveChangedSlot);
    filter_layout->addWidget(filter_ti_box, row, 0);

    QLineEdit* filter_ti_edit = new QLineEdit();
    filter_ti_edit->setText(label_generator_.filterTIValues().c_str());
    connect(filter_ti_edit, &QLineEdit::textEdited,
            this, &LabelGeneratorWidget::filterTIChangedSlot);
    filter_layout->addWidget(filter_ti_edit, row, 1);

    // ta
    ++row;
    QCheckBox* filter_ta_box = new QCheckBox("Aircraft Address");
    filter_ta_box->setChecked(label_generator_.filterTAActive());
    connect(filter_ta_box, &QCheckBox::clicked,
            this, &LabelGeneratorWidget::filterTAActiveChangedSlot);
    filter_layout->addWidget(filter_ta_box, row, 0);

    QLineEdit* filter_ta_edit = new QLineEdit();
    filter_ta_edit->setText(label_generator_.filterTAValues().c_str());
    connect(filter_ta_edit, &QLineEdit::textEdited,
            this, &LabelGeneratorWidget::filterTAChangedSlot);
    filter_layout->addWidget(filter_ta_edit, row, 1);

    // psr only
    ++row;
    QCheckBox* filter_psr_box = new QCheckBox("Primary Only");
    filter_psr_box->setChecked(label_generator_.filterPrimaryOnlyActive());
    connect(filter_psr_box, &QCheckBox::clicked,
            this, &LabelGeneratorWidget::filterPSROnlyActiveChangedSlot);
    filter_layout->addWidget(filter_psr_box, row, 0);

    filter_frame->setLayout(filter_layout);

    //main_layout->addWidget(filter_frame);
    main_layout->addWidget(scroll_area2);

    //main_layout->addStretch();

    setLayout(main_layout);
}

LabelGeneratorWidget::~LabelGeneratorWidget()
{
}

void LabelGeneratorWidget::editSettingsSlot()
{
    QMenu menu;

    QAction* action_all = new QAction("Label All", this);
    connect (action_all, &QAction::triggered, this, &LabelGeneratorWidget::labelAllDSSlot);
    menu.addAction(action_all);

    QAction* action_none = new QAction("Label None", this);
    connect (action_none, &QAction::triggered, this, &LabelGeneratorWidget::labelNoDSSlot);
    menu.addAction(action_none);

    menu.addSeparator();

    QAction* action_utn = new QAction("Use UTN as Identification", this);
    action_utn->setCheckable(true);
    action_utn->setChecked(label_generator_.useUTN());
    connect (action_utn, &QAction::triggered, this, &LabelGeneratorWidget::toggleUseUTNSlot);
    menu.addAction(action_utn);

    menu.addSeparator();

    for (auto& db_cont_it : COMPASS::instance().dbContentManager())
    {
        QAction* action = new QAction(("Edit "+db_cont_it.first).c_str(), this);
        connect (action, &QAction::triggered, this, &LabelGeneratorWidget::editDBContentSlot);
        action->setProperty("dbcontent_name", db_cont_it.first.c_str());
        menu.addAction(action);
    }

    menu.exec(QCursor::pos());
}

void LabelGeneratorWidget::editDBContentSlot()
{
    QVariant name = sender()->property("dbcontent_name");

    string dbcontent_name = name.toString().toStdString();

    loginf << "dbcontent " << dbcontent_name;

    label_generator_.editLabelContents(dbcontent_name);
}

void LabelGeneratorWidget::toggleUseUTNSlot()
{
    loginf;

    label_generator_.toggleUseUTN();
}


void LabelGeneratorWidget::labelAllDSSlot()
{
    loginf;

    label_generator_.labelAllDSIDs();

    if (label_ds_widget_)
        label_ds_widget_->forceUpdateList();
}

void LabelGeneratorWidget::labelNoDSSlot()
{
    loginf;

    label_generator_.labelNoDSIDs();

    if (label_ds_widget_)
        label_ds_widget_->forceUpdateList();
}

void LabelGeneratorWidget::autoLabelChangedSlot(bool checked)
{
    loginf << "checked " << checked;

    label_generator_.autoLabel(checked);
}

void LabelGeneratorWidget::lodChangedSlot(const QString& text)
{
    string lod = text.toStdString();

    loginf << "value " << lod;

    if (lod == "Auto")
        label_generator_.autoLOD(true);
    else
    {
        label_generator_.autoLOD(false);
        label_generator_.currentLOD(stoul(lod));
    }
}

void LabelGeneratorWidget::declutterLabelsChangedSlot(bool checked)
{
    loginf << "checked " << checked;

    label_generator_.declutterLabels(checked);
}

void LabelGeneratorWidget::filterMode3AActiveChangedSlot(bool checked)
{
    loginf << "checked " << checked;

    label_generator_.filterMode3aActive(checked);
}
void LabelGeneratorWidget::filterMode3AChangedSlot(const QString& text)
{
    string values = text.toStdString();

    loginf << "value " << values;

    label_generator_.filterMode3aValues(values);
}

void LabelGeneratorWidget::filterModeCMinActiveChangedSlot(bool checked)
{
    loginf << "checked " << checked;

    label_generator_.filterModecMinActive(checked);
}
void LabelGeneratorWidget::filterModeCMinChangedSlot(const QString& text)
{
    bool ok;

    float value = text.toFloat(&ok);

    if (ok)
        label_generator_.filterModecMinValue(value);
    else
        loginf << "impossible value '"
               << text.toStdString() << "'";

}
void LabelGeneratorWidget::filterModeCMaxActiveChangedSlot(bool checked)
{
    loginf << "checked " << checked;

    label_generator_.filterModecMaxActive(checked);
}
void LabelGeneratorWidget::filterModeCMaxChangedSlot(const QString& text)
{
    bool ok;

    float value = text.toFloat(&ok);

    if (ok)
        label_generator_.filterModecMaxValue(value);
    else
        loginf << "impossible value '"
               << text.toStdString() << "'";
}
void LabelGeneratorWidget::filterModeCNullWantedChangedSlot(bool checked)
{
    loginf << "checked " << checked;
    label_generator_.filterModecNullWanted(checked);
}

void LabelGeneratorWidget::filterTIActiveChangedSlot(bool checked)
{
    loginf << "checked " << checked;

    label_generator_.filterTIActive(checked);
}
void LabelGeneratorWidget::filterTIChangedSlot(const QString& text)
{
    string values = text.toStdString();

    loginf << "value " << values;

    label_generator_.filterTIValues(values);
}

void LabelGeneratorWidget::filterTAActiveChangedSlot(bool checked)
{
    loginf << "checked " << checked;

    label_generator_.filterTAActive(checked);
}
void LabelGeneratorWidget::filterTAChangedSlot(const QString& text)
{
    string values = text.toStdString();

    loginf << "value " << values;

    label_generator_.filterTAValues(values);
}

void LabelGeneratorWidget::filterPSROnlyActiveChangedSlot(bool checked)
{
    loginf << "checked " << checked;

    label_generator_.filterPrimaryOnlyActive(checked);
}

void LabelGeneratorWidget::opacitySliderChangedSlot(int value)
{
    logdbg << "value " << value;

    float new_op = 1 - ((float) value) / 100.0;

    logdbg << "new_op " << new_op;

    label_generator_.labelOpacity(new_op);
}

}
