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
#include "taskmanager.h"
#include "analyzedatasourcetask.h"
#include "datasourceinspectorbase.h"

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
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);

        // inspector page with the unmet prerequisite on top
        auto* page        = new QWidget();
        auto* page_layout = new QVBoxLayout(page);
        page_layout->setContentsMargins(0, 0, 0, 0);

        auto* reason_label = new QLabel();
        reason_label->setWordWrap(true);
        reason_label->setStyleSheet("QLabel { color: #b00000; }");
        page_layout->addWidget(reason_label);
        page_layout->addWidget(buildInspectorWidget(ins), 1);

        stack_->insertWidget(stack_idx++, wrapInScroll(page));
        inspector_rows_.push_back({item, ins, reason_label});
        updateInspectorRow(inspector_rows_.back());
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

    auto& ctx = task_.manager().dbContextManager();

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
        refreshInspectorRows();
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
                refreshInspectorRows();
                updateRunEnabled();
            });
    ref_line_row->addWidget(ref_line_combo_, 1);
    ref_layout->addLayout(ref_line_row);

    outer->addWidget(ref_box, 1);

    // -- Test Data --------------------------------------------------------
    auto* tst_box    = new QGroupBox("Test Data");
    auto* tst_layout = new QVBoxLayout(tst_box);

    // Candidates follow the task's DSType rule (context DSType, or for SMR:
    // Radar sources with CAT010 data).
    std::set<unsigned int> tst_cands = task_.testDataSourceCandidateIDs();

    auto* tst_list = new QListWidget();
    populateDSList(
        tst_list,
        tst_cands,
        ctx,
        [this](unsigned int ds_id) { return task_.useDataSource(ds_id); },
        QString("No ") + QString::fromStdString(task_.testDataSourceRequirementText())
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
                refreshInspectorRows();
                updateSuggestedReportName();
                updateRunEnabled();
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

    // data exclusions of the evaluation: target use flags, excluded time windows
    auto* eval_excl_cb = new QCheckBox();
    eval_excl_cb->setChecked(task_.useEvalExclusions());
    eval_excl_cb->setToolTip("Leave out the targets and time windows excluded in the evaluation "
                             "(targets not used, excluded time windows per target and global)");
    connect(eval_excl_cb, &QCheckBox::toggled, this, [this](bool on) {
        if (updating_ui_) return;
        task_.setUseEvalExclusions(on);
    });
    scope_form->addRow("Use Evaluation Exclusions", eval_excl_cb);

    // Limit by sectors: precise per-report inside test (as in the evaluation)
    // against the selected sector layers; when off, all data is used.
    {
        const auto& layers = task_.manager().dbContextManager().sectorLayers();

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

    inspector->addSettingsRows(form);

    layout->addStretch(1);
    return w;
}

void AnalyzeDataSourceDialog::refreshInspectorRows()
{
    updating_ui_ = true;
    for (auto& row : inspector_rows_)
        updateInspectorRow(row);
    updating_ui_ = false;
}

void AnalyzeDataSourceDialog::updateInspectorRow(InspectorRow& row)
{
    auto* ins  = row.inspector;
    auto* item = row.item;

    std::string reason;
    bool ok = ins->prerequisitesMet(reason);

    // An inspector with unmet prerequisites shows unchecked and disabled. The
    // task keeps the user's enabled flag, so the check returns once the
    // prerequisites are met again (e.g. after a test data source is selected).
    item->setDisabled(!ok);
    item->setCheckState(0, ok && task_.inspectorEnabled(ins->className())
                               ? Qt::Checked : Qt::Unchecked);

    // column 1: short tag, the full reason is in the tooltip and on the page
    QString tag = ins->requiresProfessionalLicense() ? "[pro]" : "";
    if (!ok)
        tag = tag.isEmpty() ? "[not available]" : tag + " [not available]";

    item->setText(1, tag);
    item->setTextAlignment(1, Qt::AlignRight | Qt::AlignVCenter);
    QFont f = item->font(1);
    f.setItalic(true);
    item->setFont(1, f);

    QString tooltip = QString::fromStdString(ok ? ins->description() : "Not available: " + reason);
    item->setToolTip(0, tooltip);
    item->setToolTip(1, tooltip);

    if (row.reason_label)
    {
        row.reason_label->setText(QString::fromStdString("Not available: " + reason));
        row.reason_label->setVisible(!ok);
    }
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

    std::string reason;
    bool can_run = task_.canRunConfiguration(&reason);

    run_button_->setEnabled(can_run);
    run_button_->setToolTip(can_run ? "Run the analysis with the current selection"
                                    : QString::fromStdString("Cannot run: " + reason));
}

void AnalyzeDataSourceDialog::runSlot()
{
    accept();
}
