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

#include "evaluationdatasourcewidget.h"
#include "logger.h"
#include "dbcontent/dbcontentcombobox.h"
#include "datasourcemanager.h"
#include "evaluationcalculator.h"

#include <QLabel>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QVariant>

using namespace std;

/**
 */
EvaluationDataSourceWidget::EvaluationDataSourceWidget(EvaluationCalculator& calculator,
                                                       const std::string& title, 
                                                       const std::string& dbcontent_name, 
                                                       unsigned int line_id,
                                                       QWidget* parent, 
                                                       Qt::WindowFlags f)
:   QFrame(parent, f)
,   calculator_(calculator)
,   title_(title)
,   dbcontent_name_(dbcontent_name)
,   line_id_(line_id)
{
    setFrameStyle(QFrame::Panel | QFrame::Raised);
    setLineWidth(2);

    QVBoxLayout* main_layout = new QVBoxLayout();

    QLabel* main_label = new QLabel(title_.c_str());
    main_layout->addWidget(main_label);

    QFrame* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    main_layout->addWidget(line);

    // dbcont
    QGridLayout* dbcont_lay = new QGridLayout();

    dbcont_lay->addWidget(new QLabel("DBContent"), 0, 0);

    dbcont_combo_ = new DBContentComboBox(false, true);
    dbcont_combo_->setObjectName(dbcontent_name_);
    connect (dbcont_combo_, &DBContentComboBox::changedObject, 
        this, &EvaluationDataSourceWidget::dbContentNameChangedSlot);

    dbcont_lay->addWidget(dbcont_combo_, 0, 1);

    main_layout->addLayout(dbcont_lay);

    // data sources
    data_source_layout_ = new QGridLayout();

    main_layout->addLayout(data_source_layout_);

    main_layout->addStretch();

    // line
    traced_assert(line_id_ <= 3);

    QGridLayout* line_lay = new QGridLayout();

    line_lay->addWidget(new QLabel("Line ID"), 0, 0);

    QComboBox* line_box = new QComboBox();
    line_box->addItems({"1", "2", "3", "4"});
    line_box->setCurrentIndex(line_id_);

    connect(line_box, &QComboBox::currentTextChanged,
            this, &EvaluationDataSourceWidget::lineIDEditSlot);
    line_lay->addWidget(line_box, 0, 1);

    main_layout->addLayout(line_lay);

    setLayout(main_layout);

    connect(&COMPASS::instance().dataSourceManager(), &DataSourceManager::dataSourcesChangedSignal,
            this, &EvaluationDataSourceWidget::updateDataSourcesSlot); // update if data sources changed
}

/**
 */
EvaluationDataSourceWidget::~EvaluationDataSourceWidget() = default;

/**
 */
void EvaluationDataSourceWidget::updateDataSourcesSlot()
{
    loginf << "title " << title_;
    traced_assert(data_source_layout_);

    QLayoutItem* child;
    while (!data_source_layout_->isEmpty() && (child = data_source_layout_->takeAt(0)) != nullptr)
    {
        if (child->widget())
            delete child->widget();
        delete child;

    }
    data_sources_checkboxes_.clear();

    unsigned int col, row;
    unsigned int cnt = 0;

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    map<string, bool> data_sources;

    if (title_ == "Reference Data")
        data_sources = static_cast<const EvaluationCalculator&>(calculator_).dataSourcesRef();
    else
        data_sources = static_cast<const EvaluationCalculator&>(calculator_).dataSourcesTst();

    unsigned int ds_id;

    for (auto& it : data_sources)
    {
        ds_id = stoul(it.first);

        if (!ds_man.hasDBDataSource(ds_id))
            continue;

        dbContent::DBDataSource& ds = ds_man.dbDataSource(ds_id);

        QCheckBox* checkbox = new QCheckBox(tr(ds.name().c_str()));
        checkbox->setChecked(it.second);
        checkbox->setProperty("id", ds_id);
        connect(checkbox, SIGNAL(clicked()), this, SLOT(toggleDataSourceSlot()));

        loginf << "got sensor " << it.first << " name "
               << ds.name() << " active " << checkbox->isChecked();

        data_sources_checkboxes_[ds_id] = checkbox;

        row = 1 + cnt / 2;
        col = cnt % 2;

        data_source_layout_->addWidget(checkbox, row, col);
        cnt++;
    }
}

/**
 */
void EvaluationDataSourceWidget::updateCheckboxesChecked()
{
    map<string, bool> data_sources;

    if (title_ == "Reference Data")
        data_sources = static_cast<const EvaluationCalculator&>(calculator_).dataSourcesRef();
    else
        data_sources = static_cast<const EvaluationCalculator&>(calculator_).dataSourcesTst();

    for (auto& checkit : data_sources_checkboxes_)
    {
        traced_assert(data_sources.count(to_string(checkit.first)));
        checkit.second->setChecked(data_sources.at(to_string(checkit.first)));
        logdbg << "ds_id " << checkit.first
               << " active " << data_sources.at(to_string(checkit.first));
    }
}

/**
 */
void EvaluationDataSourceWidget::dbContentNameChangedSlot()
{
    traced_assert(dbcont_combo_);

    dbcontent_name_ = dbcont_combo_->getObjectName();

    loginf << "name " << dbcontent_name_;

    emit dbContentNameChangedSignal(dbcontent_name_);

    updateDataSourcesSlot();

    updateCheckboxesChecked();
}

/**
 */
void EvaluationDataSourceWidget::toggleDataSourceSlot()
{
    logdbg;
    QCheckBox* check = (QCheckBox*)sender();

    unsigned int ds_id = check->property("id").toInt();

    if (title_ == "Reference Data")
    {
        loginf << "ref id " << ds_id << " checked "
               << (check->checkState() == Qt::Checked);

        auto ds_name = to_string(ds_id);
        calculator_.selectDataSourceRef(ds_name, check->checkState() == Qt::Checked);
    }
    else
    {
        loginf << "tst id " << ds_id << " checked "
               << (check->checkState() == Qt::Checked);

        auto ds_name = to_string(ds_id);
        calculator_.selectDataSourceTst(ds_name, check->checkState() == Qt::Checked);
    }

    updateCheckboxesChecked();

    emit usedDataSourceChangedSignal();
}

/**
 */
void EvaluationDataSourceWidget::lineIDEditSlot(const QString& text)
{
    bool ok;
    unsigned int line_id = text.toInt(&ok);

    traced_assert(ok);
    traced_assert(line_id <= 4);
    --line_id;

    emit lineChangedSignal(line_id);
}
