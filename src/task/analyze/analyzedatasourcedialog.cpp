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

#include "analyzedatasourcedialog.h"
#include "analyzedatasourcetask.h"
#include "datasourceinspectorbase.h"
#include "mlatdataiteminspector.h"
#include "mlatcoverageinspector.h"
#include "adsbdataiteminspector.h"
#include "adsbcoverageinspector.h"

#if USE_EXPERIMENTAL_SOURCE == true
#include "mlataccuracyinspector.h"
#include "adsbaccuracyinspector.h"
#endif

#include "compass.h"
#include "db_context_manager.h"
#include "sectorlayer.h"
#include "data_source.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QSplitter>
#include <QStackedWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

AnalyzeDataSourceDialog::AnalyzeDataSourceDialog(AnalyzeDataSourceTask& task, QWidget* parent)
    : QDialog(parent), task_(task)
{
    setWindowTitle(QString::fromStdString("Analyze " + task_.dsType() + " Data Source"));
    setMinimumSize(900, 720);

    auto* main_layout = new QVBoxLayout(this);

    auto* splitter = new QSplitter(Qt::Horizontal, this);

    tree_ = new QTreeWidget(splitter);
    tree_->setHeaderHidden(true);
    tree_->setColumnCount(2);
    tree_->setRootIsDecorated(false);
    tree_->setSelectionBehavior(QAbstractItemView::SelectRows);
    tree_->setSelectionMode(QAbstractItemView::SingleSelection);
    tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    tree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    splitter->addWidget(tree_);

    stack_ = new QStackedWidget(splitter);
    splitter->addWidget(stack_);

    tree_->setMinimumWidth(180);
    splitter->setChildrenCollapsible(false);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes({400, 600});

    main_layout->addWidget(splitter, 1);

    auto* button_row = new QHBoxLayout();

    auto* cancel = new QPushButton("Cancel", this);
    cancel->setIcon(QIcon());
    cancel->setToolTip("Close the dialog without running the analysis");
    connect(cancel, &QPushButton::clicked, this, &QDialog::reject);
    button_row->addWidget(cancel);
    button_row->addStretch(1);

    run_button_ = new QPushButton("Run", this);
    run_button_->setIcon(QIcon());
    run_button_->setToolTip("Run the analysis with the current selection");
    connect(run_button_, &QPushButton::clicked, this, &AnalyzeDataSourceDialog::runSlot);
    button_row->addWidget(run_button_);

    main_layout->addLayout(button_row);

    buildTree();

    connect(tree_, &QTreeWidget::itemSelectionChanged,
            this, &AnalyzeDataSourceDialog::treeSelectionChangedSlot);
    connect(tree_, &QTreeWidget::itemChanged,
            this, &AnalyzeDataSourceDialog::treeItemChangedSlot);

    if (ds_item_)
        ds_item_->setSelected(true);

    updateRunEnabled();
}

namespace
{
QWidget* wrapInScroll(QWidget* content)
{
    auto* scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setWidget(content);
    return scroll;
}
}

void AnalyzeDataSourceDialog::buildTree()
{
    updating_ui_ = true;

    int stack_idx = 0;

    ds_item_ = new QTreeWidgetItem(tree_);
    ds_item_->setText(0, "Data Sources");
    ds_stack_idx_ = stack_idx;
    stack_->insertWidget(stack_idx++, wrapInScroll(buildDataSourcesWidget()));

    for (const auto& ins_uptr : task_.inspectors())
    {
        DataSourceInspectorBase* ins = ins_uptr.get();

        auto* item = new QTreeWidgetItem(tree_);
        item->setText(0, QString::fromStdString(ins->name()));
        if (ins->requiresProfessionalLicense())
        {
            item->setText(1, "[pro]");
            item->setTextAlignment(1, Qt::AlignRight | Qt::AlignVCenter);
            QFont f = item->font(1);
            f.setItalic(true);
            item->setFont(1, f);
        }
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(0, task_.inspectorEnabled(ins->className())
                                   ? Qt::Checked : Qt::Unchecked);

        std::string reason;
        bool ok = ins->prerequisitesMet(reason);
        if (!ok)
        {
            item->setDisabled(true);
            item->setToolTip(0, QString::fromStdString(reason));
        }
        else if (!ins->description().empty())
        {
            item->setToolTip(0, QString::fromStdString(ins->description()));
        }

        stack_->insertWidget(stack_idx++, wrapInScroll(buildInspectorWidget(ins)));
        inspector_rows_.push_back({item, ins});
    }

    report_item_ = new QTreeWidgetItem(tree_);
    report_item_->setText(0, "Report");
    report_stack_idx_ = stack_idx;
    stack_->insertWidget(stack_idx++, wrapInScroll(buildReportWidget()));

    tree_->setCurrentItem(ds_item_);

    updating_ui_ = false;
}

namespace
{

void populateDSList(QListWidget* list,
                    const std::set<unsigned int>& ds_ids,
                    context::DBContextManager& ctx,
                    const std::function<bool(unsigned int)>& checked_fn,
                    const QString& empty_msg)
{
    bool found_any = false;
    for (auto ds_id : ds_ids)
    {
        const auto* ds = ctx.dataSource(ds_id);
        if (!ds)
            continue;

        std::string label = ds->name() + " (" + std::to_string(ds->sac()) + "/"
                            + std::to_string(ds->sic()) + ")";
        auto* item = new QListWidgetItem(QString::fromStdString(label), list);
        item->setData(Qt::UserRole, ds_id);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(checked_fn(ds_id) ? Qt::Checked : Qt::Unchecked);
        found_any = true;
    }

    if (!found_any)
    {
        auto* placeholder = new QListWidgetItem(empty_msg, list);
        placeholder->setFlags(Qt::NoItemFlags);
    }
}

}

QWidget* AnalyzeDataSourceDialog::buildDataSourcesWidget()
{
    auto* w = new QWidget();
    auto* outer = new QVBoxLayout(w);

    auto& ctx = task_.compass().dbContextManager();
    auto types = ctx.dsTypes();

    // -- Reference Data ----------------------------------------------------
    auto* ref_box    = new QGroupBox("Reference Data");
    auto* ref_layout = new QVBoxLayout(ref_box);

    auto* ref_list = new QListWidget();
    populateDSList(
        ref_list,
        task_.referenceDataSourceCandidateIDs(),
        ctx,
        [this](unsigned int ds_id) { return task_.useReferenceDataSource(ds_id); },
        "No RefTraj data sources present.");
    connect(ref_list, &QListWidget::itemChanged, this, [this](QListWidgetItem* it) {
        if (updating_ui_)
            return;
        bool ok = false;
        unsigned int ds_id = it->data(Qt::UserRole).toUInt(&ok);
        if (!ok)
            return;
        task_.useReferenceDataSource(ds_id, it->checkState() == Qt::Checked);
        updateRunEnabled();
    });
    ref_layout->addWidget(ref_list, 1);

    auto setListCheckedAll = [](QListWidget* list, Qt::CheckState state) {
        for (int i = 0; i < list->count(); ++i)
        {
            auto* it = list->item(i);
            if (it->flags() & Qt::ItemIsUserCheckable)
                it->setCheckState(state);
        }
    };

    auto* ref_btn_row = new QHBoxLayout();
    auto* ref_sel_all = new QPushButton("Select All");
    ref_sel_all->setIcon(QIcon());
    ref_sel_all->setToolTip("Select all reference data sources");
    connect(ref_sel_all, &QPushButton::clicked, this,
            [ref_list, setListCheckedAll]() {
                setListCheckedAll(ref_list, Qt::Checked);
            });
    auto* ref_sel_none = new QPushButton("Select None");
    ref_sel_none->setIcon(QIcon());
    ref_sel_none->setToolTip("Deselect all reference data sources");
    connect(ref_sel_none, &QPushButton::clicked, this,
            [ref_list, setListCheckedAll]() {
                setListCheckedAll(ref_list, Qt::Unchecked);
            });
    ref_btn_row->addWidget(ref_sel_all);
    ref_btn_row->addStretch(1);
    ref_btn_row->addWidget(ref_sel_none);
    ref_layout->addLayout(ref_btn_row);

    auto* ref_line_row = new QHBoxLayout();
    ref_line_row->addWidget(new QLabel("Line"));
    ref_line_combo_ = new QComboBox();
    ref_line_combo_->addItems({"L1", "L2", "L3", "L4"});
    ref_line_combo_->setCurrentIndex(static_cast<int>(task_.lineIDRef()));
    connect(ref_line_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int idx) {
                if (updating_ui_ || idx < 0)
                    return;
                task_.setLineIDRef(static_cast<unsigned int>(idx));
            });
    ref_line_row->addWidget(ref_line_combo_, 1);
    ref_layout->addLayout(ref_line_row);

    outer->addWidget(ref_box, 1);

    // -- Test Data --------------------------------------------------------
    auto* tst_box    = new QGroupBox("Test Data");
    auto* tst_layout = new QVBoxLayout(tst_box);

    std::set<unsigned int> tst_cands;
    for (auto ds_id : ctx.allDataSourceIds())
    {
        auto it = types.find(ds_id);
        if (it != types.end() && it->second == task_.dsType())
            tst_cands.insert(ds_id);
    }

    auto* tst_list = new QListWidget();
    populateDSList(
        tst_list,
        tst_cands,
        ctx,
        [this](unsigned int ds_id) { return task_.useDataSource(ds_id); },
        QString("No data sources of type ") + QString::fromStdString(task_.dsType())
            + " present.");
    connect(tst_list, &QListWidget::itemChanged, this, [this](QListWidgetItem* it) {
        if (updating_ui_)
            return;
        bool ok = false;
        unsigned int ds_id = it->data(Qt::UserRole).toUInt(&ok);
        if (!ok)
            return;
        task_.useDataSource(ds_id, it->checkState() == Qt::Checked);
        // task_.useDataSource resets the custom name on a real change; clear
        // the dialog's edit flag too so updateSuggestedReportName actually
        // pushes the new default into the line edit.
        report_name_user_edited_ = task_.hasCustomReportName();
        refreshInspectorRows();
        updateSuggestedReportName();
        updateRunEnabled();
    });
    tst_layout->addWidget(tst_list, 1);

    auto* tst_btn_row = new QHBoxLayout();
    auto* tst_sel_all = new QPushButton("Select All");
    tst_sel_all->setIcon(QIcon());
    tst_sel_all->setToolTip("Select all test data sources");
    connect(tst_sel_all, &QPushButton::clicked, this,
            [tst_list, setListCheckedAll]() {
                setListCheckedAll(tst_list, Qt::Checked);
            });
    auto* tst_sel_none = new QPushButton("Select None");
    tst_sel_none->setIcon(QIcon());
    tst_sel_none->setToolTip("Deselect all test data sources");
    connect(tst_sel_none, &QPushButton::clicked, this,
            [tst_list, setListCheckedAll]() {
                setListCheckedAll(tst_list, Qt::Unchecked);
            });
    tst_btn_row->addWidget(tst_sel_all);
    tst_btn_row->addStretch(1);
    tst_btn_row->addWidget(tst_sel_none);
    tst_layout->addLayout(tst_btn_row);

    auto* tst_line_row = new QHBoxLayout();
    tst_line_row->addWidget(new QLabel("Line"));
    tst_line_combo_ = new QComboBox();
    tst_line_combo_->addItems({"L1", "L2", "L3", "L4"});
    tst_line_combo_->setCurrentIndex(static_cast<int>(task_.lineIDTst()));
    connect(tst_line_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int idx) {
                if (updating_ui_ || idx < 0)
                    return;
                task_.setLineIDTst(static_cast<unsigned int>(idx));
                updateSuggestedReportName();
            });
    tst_line_row->addWidget(tst_line_combo_, 1);
    tst_layout->addLayout(tst_line_row);

    outer->addWidget(tst_box, 1);

    // -- Scope Filter ------------------------------------------------------
    // Per-report inside test applied when the combined dataset is built;
    // restricts every grid inspector to on-ground reports and/or a flight-level
    // band (FL = barometric altitude / 100).
    auto* scope_box    = new QGroupBox("Scope Filter");
    auto* scope_form   = new QFormLayout(scope_box);

    auto* ground_cb = new QCheckBox();
    ground_cb->setChecked(task_.useGroundOnly());
    ground_cb->setToolTip("Keep only reports that are on the ground "
                          "(ground bit set, or a ground-only target type).");
    connect(ground_cb, &QCheckBox::toggled, this, [this](bool on) {
        if (updating_ui_) return;
        task_.setUseGroundOnly(on);
    });
    scope_form->addRow("Use Ground Only", ground_cb);

    auto makeFLSpin = [](float value) {
        auto* s = new QDoubleSpinBox();
        s->setRange(0.0, 999.0);
        s->setDecimals(0);
        s->setSingleStep(5.0);
        s->setValue(value);
        return s;
    };

    auto* min_fl_cb   = new QCheckBox();
    min_fl_cb->setChecked(task_.useMinFL());
    auto* min_fl_spin = makeFLSpin(task_.minFL());
    min_fl_spin->setEnabled(task_.useMinFL());
    min_fl_cb->setToolTip("Keep only reports at or above this flight level.");
    connect(min_fl_cb, &QCheckBox::toggled, this, [this, min_fl_spin](bool on) {
        if (updating_ui_) return;
        task_.setUseMinFL(on);
        min_fl_spin->setEnabled(on);
    });
    connect(min_fl_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double v) {
        if (updating_ui_) return;
        task_.setMinFL(static_cast<float>(v));
    });
    {
        auto* row = new QHBoxLayout();
        row->addWidget(min_fl_cb);
        row->addWidget(min_fl_spin, 1);
        scope_form->addRow("Minimum Flight Level", row);
    }

    auto* max_fl_cb   = new QCheckBox();
    max_fl_cb->setChecked(task_.useMaxFL());
    auto* max_fl_spin = makeFLSpin(task_.maxFL());
    max_fl_spin->setEnabled(task_.useMaxFL());
    max_fl_cb->setToolTip("Keep only reports at or below this flight level.");
    connect(max_fl_cb, &QCheckBox::toggled, this, [this, max_fl_spin](bool on) {
        if (updating_ui_) return;
        task_.setUseMaxFL(on);
        max_fl_spin->setEnabled(on);
    });
    connect(max_fl_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double v) {
        if (updating_ui_) return;
        task_.setMaxFL(static_cast<float>(v));
    });
    {
        auto* row = new QHBoxLayout();
        row->addWidget(max_fl_cb);
        row->addWidget(max_fl_spin, 1);
        scope_form->addRow("Maximum Flight Level", row);
    }

    // Limit by sectors: precise per-report inside test (as in the evaluation)
    // against the selected sector layers; when off, all data is used.
    {
        const auto& layers = task_.compass().dbContextManager().sectorLayers();

        auto* sectors_cb = new QCheckBox();
        sectors_cb->setChecked(task_.limitBySectors());
        sectors_cb->setEnabled(!layers.empty());
        sectors_cb->setToolTip(layers.empty()
            ? "No sector layers are defined in the data context."
            : "Keep only reports inside the selected sector layers.");
        scope_form->addRow("Limit by Sectors", sectors_cb);

        auto* sectors_box  = new QGroupBox("Sectors");
        auto* sectors_lay  = new QVBoxLayout(sectors_box);
        sectors_box->setEnabled(task_.limitBySectors() && !layers.empty());

        if (layers.empty())
        {
            sectors_lay->addWidget(new QLabel("(none defined)"));
        }
        else
        {
            for (const auto& layer : layers)
            {
                if (!layer)
                    continue;
                const std::string name = layer->name();
                auto* cb = new QCheckBox(QString::fromStdString(name));
                cb->setChecked(task_.useSector(name));
                connect(cb, &QCheckBox::toggled, this, [this, name](bool on) {
                    if (updating_ui_) return;
                    task_.setUseSector(name, on);
                    updateRunEnabled();
                });
                sectors_lay->addWidget(cb);
            }
        }

        connect(sectors_cb, &QCheckBox::toggled, this, [this, sectors_box](bool on) {
            if (updating_ui_) return;
            task_.setLimitBySectors(on);
            sectors_box->setEnabled(on);
            updateRunEnabled();
        });

        scope_form->addRow(sectors_box);
    }

    outer->addWidget(scope_box);

    return w;
}

QWidget* AnalyzeDataSourceDialog::buildReportWidget()
{
    auto* w = new QWidget();
    auto* form = new QFormLayout(w);

    report_name_edit_ = new QLineEdit();
    report_name_edit_->setText(QString::fromStdString(task_.reportName()));
    report_name_user_edited_ = task_.hasCustomReportName();
    connect(report_name_edit_, &QLineEdit::textEdited,
            this, [this](const QString& text) {
                report_name_user_edited_ = true;
                task_.setCustomReportName(text.toStdString());
            });
    form->addRow("Report Name", report_name_edit_);

    auto* h_spin = new QDoubleSpinBox();
    h_spin->setRange(1.0, 100000.0);
    h_spin->setDecimals(1);
    h_spin->setSingleStep(10.0);
    h_spin->setSuffix(" m");
    h_spin->setValue(task_.cellSizeMeters());
    connect(h_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double v) {
                if (updating_ui_) return;
                task_.setCellSizeMeters(static_cast<float>(v));
            });
    form->addRow("Horizontal cell size", h_spin);

    auto* v_spin = new QDoubleSpinBox();
    v_spin->setRange(1.0, 100000.0);
    v_spin->setDecimals(1);
    v_spin->setSingleStep(50.0);
    v_spin->setSuffix(" ft");
    v_spin->setValue(task_.cellSizeFeet());
    connect(v_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double v) {
                if (updating_ui_) return;
                task_.setCellSizeFeet(static_cast<float>(v));
            });
    form->addRow("Vertical cell size", v_spin);

    auto* max_spin = new QSpinBox();
    max_spin->setRange(10, 2000);
    max_spin->setSingleStep(50);
    max_spin->setValue(static_cast<int>(task_.maxCellsPerAxis()));
    connect(max_spin, QOverload<int>::of(&QSpinBox::valueChanged),
            this, [this](int v) {
                if (updating_ui_) return;
                task_.setMaxCellsPerAxis(static_cast<unsigned int>(v));
            });
    form->addRow("Max cells per axis", max_spin);

    return w;
}

void AnalyzeDataSourceDialog::updateSuggestedReportName()
{
    if (report_name_user_edited_ || !report_name_edit_)
        return;
    QSignalBlocker blocker(report_name_edit_);
    report_name_edit_->setText(QString::fromStdString(task_.suggestReportName()));
}

namespace
{

QDoubleSpinBox* makeFloatSpin(double min, double max, double step, int decimals,
                              const QString& suffix, double value)
{
    auto* s = new QDoubleSpinBox();
    s->setRange(min, max);
    s->setSingleStep(step);
    s->setDecimals(decimals);
    if (!suffix.isEmpty())
        s->setSuffix(suffix);
    s->setValue(value);
    return s;
}

void buildDataItemSettings(QFormLayout* form,
                           AnalyzeDataSourceTask& task,
                           MLATDataItemInspectorSettings& s)
{
    auto* group = new QGroupBox("Categories to Include");
    auto* gl = new QVBoxLayout(group);

    std::set<unsigned int> cats;
    auto& info_map = task.compass().dbContextManager().asterixInfo();
    for (auto ds_id : task.selectedDataSourceIDs())
    {
        auto it = info_map.find(ds_id);
        if (it == info_map.end())
            continue;
        for (const auto& cat_kv : it->second)
            cats.insert(cat_kv.first);
    }

    if (cats.empty())
    {
        auto* lbl = new QLabel(
            "(no ASTERIX info available - select test data sources first)");
        lbl->setStyleSheet("color: gray;");
        gl->addWidget(lbl);
    }
    else
    {
        for (auto cat : cats)
        {
            char buf[16];
            std::snprintf(buf, sizeof(buf), "CAT%03u", cat);
            auto* cb = new QCheckBox(buf);
            cb->setChecked(s.catIncluded(cat));
            QObject::connect(cb, &QCheckBox::toggled, [&s, cat](bool on) {
                s.setCatIncluded(cat, on);
            });
            gl->addWidget(cb);
        }
    }

    form->addRow(group);
}

void buildCoverageSettings(QFormLayout* form,
                           MLATCoverageInspectorSettings& s)
{
    auto* method_combo = new QComboBox();
    method_combo->addItem("Time Difference",
                          static_cast<int>(MLATCoverageInspectorSettings::PDMethod::TimeDifference));
    method_combo->addItem("Status Period Message Based",
                          static_cast<int>(MLATCoverageInspectorSettings::PDMethod::StatusPeriodMessage));
    int sel_idx = method_combo->findData(s.pd_method_int_);
    method_combo->setCurrentIndex(sel_idx >= 0 ? sel_idx : 0);

    auto* ui_spin = makeFloatSpin(0.05, 60.0, 0.1, 2, " s", s.update_interval_s_);

    auto on_method = [&s, ui_spin, method_combo](int) {
        s.pd_method_int_ = method_combo->currentData().toInt();
        ui_spin->setEnabled(s.pdMethod() ==
                            MLATCoverageInspectorSettings::PDMethod::TimeDifference);
    };
    QObject::connect(method_combo,
                     QOverload<int>::of(&QComboBox::currentIndexChanged),
                     on_method);
    on_method(0);

    QObject::connect(ui_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     [&s](double v) { s.update_interval_s_ = static_cast<float>(v); });

    auto* ui_stand_spin = makeFloatSpin(0.05, 60.0, 0.1, 2, " s", s.update_interval_standing_s_);
    QObject::connect(ui_stand_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     [&s](double v) { s.update_interval_standing_s_ = static_cast<float>(v); });

    auto* stand_spd_spin = makeFloatSpin(0.0, 50.0, 0.1, 2, " m/s", s.standing_speed_max_mps_);
    QObject::connect(stand_spd_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     [&s](double v) { s.standing_speed_max_mps_ = static_cast<float>(v); });

    // Standing UI/threshold only apply to the time-difference method.
    auto on_method_standing = [ui_stand_spin, stand_spd_spin, method_combo](int) {
        bool td = method_combo->currentData().toInt()
                  == static_cast<int>(MLATCoverageInspectorSettings::PDMethod::TimeDifference);
        ui_stand_spin->setEnabled(td);
        stand_spd_spin->setEnabled(td);
    };
    QObject::connect(method_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
                     on_method_standing);
    on_method_standing(0);

    form->addRow("PD Calculation Method", method_combo);
    form->addRow("Update Interval (Moving)", ui_spin);
    form->addRow("Update Interval (Standing)", ui_stand_spin);
    form->addRow("Standing Speed Threshold", stand_spd_spin);

    auto* miss_cb  = new QCheckBox();
    miss_cb->setChecked(s.use_miss_tolerance_);
    auto* miss_spin = makeFloatSpin(0.0, 60.0, 0.05, 3, " s", s.miss_tolerance_s_);
    miss_spin->setEnabled(s.use_miss_tolerance_);
    QObject::connect(miss_cb, &QCheckBox::toggled, [&s, miss_spin](bool on) {
        s.use_miss_tolerance_ = on;
        miss_spin->setEnabled(on);
    });
    QObject::connect(miss_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     [&s](double v) { s.miss_tolerance_s_ = static_cast<float>(v); });
    form->addRow("Use Miss Tolerance", miss_cb);
    form->addRow("Miss Tolerance",     miss_spin);

    auto* acc_spin = makeFloatSpin(0.0, 1.0, 0.01, 2, "", s.pd_acceptable_above_);
    QObject::connect(acc_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     [&s](double v) { s.pd_acceptable_above_ = static_cast<float>(v); });
    form->addRow("Value Acceptable (green)", acc_spin);

    auto* unacc_spin = makeFloatSpin(0.0, 1.0, 0.01, 2, "", s.pd_unacceptable_below_);
    QObject::connect(unacc_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     [&s](double v) { s.pd_unacceptable_below_ = static_cast<float>(v); });
    form->addRow("Value Unacceptable (red)", unacc_spin);

    auto* hbins_spin = new QSpinBox();
    hbins_spin->setRange(1, 100);
    hbins_spin->setValue(s.ui_hist_num_bins_);
    QObject::connect(hbins_spin, QOverload<int>::of(&QSpinBox::valueChanged),
                     [&s](int v) { s.ui_hist_num_bins_ = v; });
    form->addRow("Histogram Bins", hbins_spin);

    auto* hmax_spin = makeFloatSpin(0.0, 600.0, 1.0, 1, " s", s.ui_hist_max_s_);
    QObject::connect(hmax_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     [&s](double v) { s.ui_hist_max_s_ = static_cast<float>(v); });
    form->addRow("Histogram Max Interval (0 = auto)", hmax_spin);
}

#if USE_EXPERIMENTAL_SOURCE == true
void buildAccuracySettings(QFormLayout* form,
                           MLATAccuracyInspectorSettings& s)
{
    auto bind_float = [](QDoubleSpinBox* spin, float* dst) {
        QObject::connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                         [dst](double v) { *dst = static_cast<float>(v); });
    };

    auto* pa_acc = makeFloatSpin(0.0, 10000.0, 1.0, 1, " m",
                                 s.pos_acc_acceptable_below_m_);
    bind_float(pa_acc, &s.pos_acc_acceptable_below_m_);
    form->addRow("Position Accuracy Acceptable (green)", pa_acc);

    auto* pa_unacc = makeFloatSpin(0.0, 10000.0, 1.0, 1, " m",
                                   s.pos_acc_unacceptable_above_m_);
    bind_float(pa_unacc, &s.pos_acc_unacceptable_above_m_);
    form->addRow("Position Accuracy Unacceptable (red)", pa_unacc);

    auto* lower_acc = makeFloatSpin(0.0, 10.0, 0.05, 2, "",
                                    s.consistency_lower_acceptable_above_);
    bind_float(lower_acc, &s.consistency_lower_acceptable_above_);
    form->addRow("Consistency Lower Acceptable (green)", lower_acc);

    auto* lower_unacc = makeFloatSpin(0.0, 10.0, 0.05, 2, "",
                                      s.consistency_lower_unacceptable_below_);
    bind_float(lower_unacc, &s.consistency_lower_unacceptable_below_);
    form->addRow("Consistency Lower Unacceptable (red)", lower_unacc);

    auto* upper_acc = makeFloatSpin(0.0, 100.0, 0.05, 2, "",
                                    s.consistency_upper_acceptable_below_);
    bind_float(upper_acc, &s.consistency_upper_acceptable_below_);
    form->addRow("Consistency Upper Acceptable (green)", upper_acc);

    auto* upper_unacc = makeFloatSpin(0.0, 100.0, 0.05, 2, "",
                                      s.consistency_upper_unacceptable_above_);
    bind_float(upper_unacc, &s.consistency_upper_unacceptable_above_);
    form->addRow("Consistency Upper Unacceptable (red)", upper_unacc);
}

void buildADSBAccuracySettings(QFormLayout* form,
                               ADSBAccuracyInspectorSettings& s)
{
    auto bind_float = [](QDoubleSpinBox* spin, float* dst) {
        QObject::connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                         [dst](double v) { *dst = static_cast<float>(v); });
    };

    auto* pa_acc = makeFloatSpin(0.0, 10000.0, 1.0, 1, " m",
                                 s.pos_acc_acceptable_below_m_);
    bind_float(pa_acc, &s.pos_acc_acceptable_below_m_);
    form->addRow("Position Accuracy Acceptable (green)", pa_acc);

    auto* pa_unacc = makeFloatSpin(0.0, 10000.0, 1.0, 1, " m",
                                   s.pos_acc_unacceptable_above_m_);
    bind_float(pa_unacc, &s.pos_acc_unacceptable_above_m_);
    form->addRow("Position Accuracy Unacceptable (red)", pa_unacc);

    auto* lower_acc = makeFloatSpin(0.0, 10.0, 0.05, 2, "",
                                    s.consistency_lower_acceptable_above_);
    bind_float(lower_acc, &s.consistency_lower_acceptable_above_);
    form->addRow("Consistency Lower Acceptable (green)", lower_acc);

    auto* lower_unacc = makeFloatSpin(0.0, 10.0, 0.05, 2, "",
                                      s.consistency_lower_unacceptable_below_);
    bind_float(lower_unacc, &s.consistency_lower_unacceptable_below_);
    form->addRow("Consistency Lower Unacceptable (red)", lower_unacc);

    auto* upper_acc = makeFloatSpin(0.0, 100.0, 0.05, 2, "",
                                    s.consistency_upper_acceptable_below_);
    bind_float(upper_acc, &s.consistency_upper_acceptable_below_);
    form->addRow("Consistency Upper Acceptable (green)", upper_acc);

    auto* upper_unacc = makeFloatSpin(0.0, 100.0, 0.05, 2, "",
                                      s.consistency_upper_unacceptable_above_);
    bind_float(upper_unacc, &s.consistency_upper_unacceptable_above_);
    form->addRow("Consistency Upper Unacceptable (red)", upper_unacc);

    auto* factor = makeFloatSpin(1.0, 100.0, 0.5, 2, "", s.min_factor_of_interest_);
    bind_float(factor, &s.min_factor_of_interest_);
    form->addRow("Dubious Factor Threshold", factor);
}
#endif

void buildADSBDataItemSettings(QFormLayout* form,
                               AnalyzeDataSourceTask& task,
                               ADSBDataItemInspectorSettings& s)
{
    auto* group = new QGroupBox("Categories to Include");
    auto* gl = new QVBoxLayout(group);

    std::set<unsigned int> cats;
    auto& info_map = task.compass().dbContextManager().asterixInfo();
    for (auto ds_id : task.selectedDataSourceIDs())
    {
        auto it = info_map.find(ds_id);
        if (it == info_map.end())
            continue;
        for (const auto& cat_kv : it->second)
            cats.insert(cat_kv.first);
    }

    if (cats.empty())
    {
        auto* lbl = new QLabel(
            "(no ASTERIX info available - select test data sources first)");
        lbl->setStyleSheet("color: gray;");
        gl->addWidget(lbl);
    }
    else
    {
        for (auto cat : cats)
        {
            char buf[16];
            std::snprintf(buf, sizeof(buf), "CAT%03u", cat);
            auto* cb = new QCheckBox(buf);
            cb->setChecked(s.catIncluded(cat));
            QObject::connect(cb, &QCheckBox::toggled, [&s, cat](bool on) {
                s.setCatIncluded(cat, on);
            });
            gl->addWidget(cb);
        }
    }

    form->addRow(group);
}

void buildADSBCoverageSettings(QFormLayout* form,
                               ADSBCoverageInspectorSettings& s)
{
    auto* ui_spin = makeFloatSpin(0.05, 60.0, 0.1, 2, " s", s.update_interval_s_);
    QObject::connect(ui_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     [&s](double v) { s.update_interval_s_ = static_cast<float>(v); });
    form->addRow("Update Interval (Moving)", ui_spin);

    auto* ui_stand_spin = makeFloatSpin(0.05, 60.0, 0.1, 2, " s", s.update_interval_standing_s_);
    QObject::connect(ui_stand_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     [&s](double v) { s.update_interval_standing_s_ = static_cast<float>(v); });
    form->addRow("Update Interval (Standing)", ui_stand_spin);

    auto* stand_spd_spin = makeFloatSpin(0.0, 50.0, 0.1, 2, " m/s", s.standing_speed_max_mps_);
    QObject::connect(stand_spd_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     [&s](double v) { s.standing_speed_max_mps_ = static_cast<float>(v); });
    form->addRow("Standing Speed Threshold", stand_spd_spin);

    auto* miss_cb  = new QCheckBox();
    miss_cb->setChecked(s.use_miss_tolerance_);
    auto* miss_spin = makeFloatSpin(0.0, 60.0, 0.05, 3, " s", s.miss_tolerance_s_);
    miss_spin->setEnabled(s.use_miss_tolerance_);
    QObject::connect(miss_cb, &QCheckBox::toggled, [&s, miss_spin](bool on) {
        s.use_miss_tolerance_ = on;
        miss_spin->setEnabled(on);
    });
    QObject::connect(miss_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     [&s](double v) { s.miss_tolerance_s_ = static_cast<float>(v); });
    form->addRow("Use Miss Tolerance", miss_cb);
    form->addRow("Miss Tolerance",     miss_spin);

    auto* acc_spin = makeFloatSpin(0.0, 1.0, 0.01, 2, "", s.pd_acceptable_above_);
    QObject::connect(acc_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     [&s](double v) { s.pd_acceptable_above_ = static_cast<float>(v); });
    form->addRow("Value Acceptable (green)", acc_spin);

    auto* unacc_spin = makeFloatSpin(0.0, 1.0, 0.01, 2, "", s.pd_unacceptable_below_);
    QObject::connect(unacc_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     [&s](double v) { s.pd_unacceptable_below_ = static_cast<float>(v); });
    form->addRow("Value Unacceptable (red)", unacc_spin);

    auto* hbins_spin = new QSpinBox();
    hbins_spin->setRange(1, 100);
    hbins_spin->setValue(s.ui_hist_num_bins_);
    QObject::connect(hbins_spin, QOverload<int>::of(&QSpinBox::valueChanged),
                     [&s](int v) { s.ui_hist_num_bins_ = v; });
    form->addRow("Histogram Bins", hbins_spin);

    auto* hmax_spin = makeFloatSpin(0.0, 600.0, 1.0, 1, " s", s.ui_hist_max_s_);
    QObject::connect(hmax_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     [&s](double v) { s.ui_hist_max_s_ = static_cast<float>(v); });
    form->addRow("Histogram Max Interval (0 = auto)", hmax_spin);
}

}

QWidget* AnalyzeDataSourceDialog::buildInspectorWidget(DataSourceInspectorBase* inspector)
{
    auto* w = new QWidget();
    auto* layout = new QVBoxLayout(w);

    auto* title = new QLabel(QString::fromStdString(inspector->name()));
    QFont f = title->font();
    f.setBold(true);
    f.setPointSize(f.pointSize() + 2);
    title->setFont(f);
    layout->addWidget(title);

    auto* form = new QFormLayout();
    layout->addLayout(form);

    const std::string cn = inspector->className();
    if (cn == "MLATDataItemInspector")
        buildDataItemSettings(form, task_, task_.dataItemSettings());
    else if (cn == "MLATCoverageInspector")
        buildCoverageSettings(form, task_.coverageSettings());
    else if (cn == "ADSBDataItemInspector")
        buildADSBDataItemSettings(form, task_, task_.adsbDataItemSettings());
    else if (cn == "ADSBCoverageInspector")
        buildADSBCoverageSettings(form, task_.adsbCoverageSettings());
#if USE_EXPERIMENTAL_SOURCE == true
    else if (cn == "MLATAccuracyInspector")
        buildAccuracySettings(form, task_.accuracySettings());
    else if (cn == "ADSBAccuracyInspector")
        buildADSBAccuracySettings(form, task_.adsbAccuracySettings());
#endif

    layout->addStretch(1);
    return w;
}

void AnalyzeDataSourceDialog::refreshInspectorRows()
{
    updating_ui_ = true;
    for (auto& row : inspector_rows_)
    {
        std::string reason;
        bool ok = row.inspector->prerequisitesMet(reason);
        row.item->setDisabled(!ok);
        row.item->setToolTip(0, QString::fromStdString(ok ? row.inspector->description()
                                                          : reason));
        // Preserve the user's check state even when prerequisites are
        // temporarily unmet (e.g. no test data sources selected); the Run
        // button gates execution.
    }
    updating_ui_ = false;
}

void AnalyzeDataSourceDialog::treeSelectionChangedSlot()
{
    auto items = tree_->selectedItems();
    if (items.isEmpty())
        return;
    auto* sel = items.first();

    if (sel == ds_item_)
    {
        stack_->setCurrentIndex(ds_stack_idx_);
        return;
    }
    if (sel == report_item_)
    {
        stack_->setCurrentIndex(report_stack_idx_);
        // Refresh shown text in case the suggestion changed while another
        // page was active.
        if (report_name_edit_ && !report_name_user_edited_)
        {
            QSignalBlocker blocker(report_name_edit_);
            report_name_edit_->setText(QString::fromStdString(task_.reportName()));
        }
        return;
    }

    for (size_t i = 0; i < inspector_rows_.size(); ++i)
    {
        if (inspector_rows_[i].item == sel)
        {
            // Inspector pages are inserted between Data Sources and Report.
            stack_->setCurrentIndex(ds_stack_idx_ + 1 + static_cast<int>(i));
            return;
        }
    }
}

void AnalyzeDataSourceDialog::treeItemChangedSlot(QTreeWidgetItem* item, int column)
{
    if (updating_ui_ || column != 0)
        return;

    for (auto& row : inspector_rows_)
    {
        if (row.item != item)
            continue;
        bool checked = item->checkState(0) == Qt::Checked;
        task_.inspectorEnabled(row.inspector->className(), checked);
        updateRunEnabled();
        return;
    }
}

void AnalyzeDataSourceDialog::updateRunEnabled()
{
    if (!run_button_)
        return;

    bool can_run = !task_.selectedDataSourceIDs().empty()
                   && !task_.selectedReferenceDataSourceIDs().empty();
    if (can_run)
    {
        bool any_enabled = false;
        for (const auto& row : inspector_rows_)
        {
            if (row.item->isDisabled())
                continue;
            if (row.item->checkState(0) == Qt::Checked
                && task_.inspectorEnabled(row.inspector->className()))
            {
                any_enabled = true;
                break;
            }
        }
        can_run = any_enabled;
    }
    // Limit by sectors requires at least one selected sector layer.
    if (can_run && task_.limitBySectors() && task_.selectedSectorLayers().empty())
        can_run = false;
    run_button_->setEnabled(can_run);
}

void AnalyzeDataSourceDialog::runSlot()
{
    accept();
}
