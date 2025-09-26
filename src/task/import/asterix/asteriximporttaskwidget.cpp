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

#include "asteriximporttaskwidget.h"
#include "asterixconfigwidget.h"
#include "asteriximporttask.h"
#include "asterixoverridewidget.h"
#include "logger.h"
#include "dbcontent/selectdialog.h"
#include "util/timeconv.h"
#include "util/files.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QFrame>
#include <QInputDialog>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QDateEdit>
#include <QTreeWidgetItem>
#include <QTreeWidget>
#include <QHeaderView>

using namespace Utils;
using namespace std;

ASTERIXImportTaskWidget::ASTERIXImportTaskWidget(ASTERIXImportTask& task, QWidget* parent,
                                                 Qt::WindowFlags f)
    : QWidget(parent, f), task_(task)
{
    main_layout_ = new QHBoxLayout();

    tab_widget_ = new QTabWidget();

    main_layout_->addWidget(tab_widget_);

    addMainTab();
    addDecoderTab();
    addOverrideTab();
    addMappingsTab();

    setLayout(main_layout_);

    connect(&task, &ASTERIXImportTask::decodingStateChanged, this, &ASTERIXImportTaskWidget::decodingStateChangedSlot);
}

void ASTERIXImportTaskWidget::addMainTab()
{
    traced_assert(tab_widget_);

    QFont font_bold;
    font_bold.setBold(true);

    QVBoxLayout* main_tab_layout = new QVBoxLayout();

    // source stuff
    {
        QFormLayout* source_layout = new QFormLayout();

        sources_grid_ = new QGridLayout();
        updateSourcesGrid();

        main_tab_layout->addLayout(sources_grid_);

        main_tab_layout->addStretch();

        if (task_.source().isNetworkType())
        {
            loginf << "is network import";
        }
        else
        {
            loginf << "is file import";

            // line
            QComboBox* file_line_box = new QComboBox();
            file_line_box->addItems({"1", "2", "3", "4"});
            file_line_box->setCurrentText(QString::number(task_.settings().file_line_id_+1)); // from 0..3

            connect(file_line_box, &QComboBox::currentTextChanged,
                    this, &ASTERIXImportTaskWidget::fileLineIDEditSlot);
            source_layout->addRow("Line ID", file_line_box);

            // date
            QDateEdit* date_edit = new QDateEdit();
            date_edit->setDisplayFormat("yyyy-MM-dd");

            //loginf << "UGA " << Time::toDateString(task_.date());

            QDate date = QDate::fromString(Time::toDateString(task_.settings().date_).c_str(), "yyyy-MM-dd");
            //loginf << "UGA2 " << date.toString().toStdString();

            date_edit->setDate(date);

            connect(date_edit, &QDateEdit::dateChanged,
                    this, &ASTERIXImportTaskWidget::dateChangedSlot);
            source_layout->addRow("UTC Day", date_edit);
        }

        main_tab_layout->addLayout(source_layout);
    }

    main_tab_layout->addStretch();

    // final stuff

    {
        reset_date_between_files_check_ = new QCheckBox("Reset Date Between Files");
        reset_date_between_files_check_->setToolTip(
            "Disable if multiple sequential files with date increments are imported");
        reset_date_between_files_check_->setChecked(task_.settings().reset_date_between_files_);
        connect(reset_date_between_files_check_, &QCheckBox::clicked,
                this, &ASTERIXImportTaskWidget::resetDateChangedSlot);
        main_tab_layout->addWidget(reset_date_between_files_check_);

        ignore_timejumps_check_ = new QCheckBox("Ignore 24h Time Jumps");
        ignore_timejumps_check_->setChecked(task_.settings().ignore_time_jumps_);
        connect(ignore_timejumps_check_, &QCheckBox::clicked, this,
                &ASTERIXImportTaskWidget::ignoreTimeJumpsCheckedSlot);
        main_tab_layout->addWidget(ignore_timejumps_check_);
    }

    {
        debug_check_ = new QCheckBox("Debug in Console");
        debug_check_->setChecked(task_.settings().debug_jasterix_);
        connect(debug_check_, &QCheckBox::clicked,
                this, &ASTERIXImportTaskWidget::debugChangedSlot);
        main_tab_layout->addWidget(debug_check_);
    }

    QWidget* main_tab_widget = new QWidget();
    main_tab_widget->setContentsMargins(0, 0, 0, 0);
    main_tab_widget->setLayout(main_tab_layout);
    tab_widget_->addTab(main_tab_widget, "Main");
}

void ASTERIXImportTaskWidget::addDecoderTab()
{
    traced_assert(tab_widget_);

    config_widget_ = new ASTERIXConfigWidget(task_, this);
    tab_widget_->addTab(config_widget_, "Decoder");
}

void ASTERIXImportTaskWidget::addOverrideTab()
{
    traced_assert(tab_widget_);

    override_widget_ = new ASTERIXOverrideWidget(task_, this);
    tab_widget_->addTab(override_widget_, "Override/Filter");
}

void ASTERIXImportTaskWidget::addMappingsTab()
{
    QVBoxLayout* parsers_layout = new QVBoxLayout();

    QHBoxLayout* parser_manage_layout = new QHBoxLayout();

    object_parser_box_ = new QComboBox();
    connect(object_parser_box_, SIGNAL(currentIndexChanged(const QString&)), this,
            SLOT(selectedObjectParserSlot(const QString&)));

    parser_manage_layout->addWidget(object_parser_box_);

    add_object_parser_button_ = new QPushButton("Add");
    connect(add_object_parser_button_, SIGNAL(clicked()), this, SLOT(addParserSlot()));
    add_object_parser_button_->setEnabled(COMPASS::instance().expertMode());
    parser_manage_layout->addWidget(add_object_parser_button_);

    delete_object_parser_button_ = new QPushButton("Remove");
    connect(delete_object_parser_button_, SIGNAL(clicked()), this, SLOT(removeObjectParserSlot()));
    delete_object_parser_button_->setEnabled(COMPASS::instance().expertMode());
    parser_manage_layout->addWidget(delete_object_parser_button_);

    parsers_layout->addLayout(parser_manage_layout);

    QFrame* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    parsers_layout->addWidget(line);

    object_parser_widget_ = new QStackedWidget();
    parsers_layout->addWidget(object_parser_widget_);

    updateParserBox();

    QWidget* mappings_tab_widget = new QWidget();
    mappings_tab_widget->setContentsMargins(0, 0, 0, 0);
    mappings_tab_widget->setLayout(parsers_layout);
    tab_widget_->addTab(mappings_tab_widget, "Mappings");
}

ASTERIXImportTaskWidget::~ASTERIXImportTaskWidget() { config_widget_ = nullptr; }

void ASTERIXImportTaskWidget::addParserSlot()
{
    if (task_.schema() == nullptr)
    {
        QMessageBox m_warning(QMessageBox::Warning, "JSON Object Parser Adding Failed",
                              "No current JSON Parsing Schema is selected.", QMessageBox::Ok);

        m_warning.exec();
        return;
    }

    dbContent::SelectDBContentDialog dialog;

    int ret = dialog.exec();

    if (ret == QDialog::Accepted)
    {
        unsigned int cat = dialog.category();
        std::string dbcontent_name = dialog.selectedObject();
        loginf << "cat " << cat << " obj "
               << dbcontent_name;

        std::shared_ptr<ASTERIXJSONParsingSchema> current = task_.schema();

        if (current->hasObjectParser(cat))
        {
            QMessageBox m_warning(QMessageBox::Warning, "ASTERIX JSON Parser Adding Failed",
                                  "ASTERIX parser for category already defined.", QMessageBox::Ok);

            m_warning.exec();
            return;
        }

        std::string instance = "ASTERIXJSONParserCAT" + to_string(cat) + "0";

        auto config = Configuration::create("ASTERIXJSONParser", instance);
        config->addParameter<unsigned int>("category", cat);
        config->addParameter<std::string>("dbcontent_name", dbcontent_name);

        current->generateSubConfigurableFromConfig(std::move(config));
        updateParserBox();
    }
}
void ASTERIXImportTaskWidget::removeObjectParserSlot()
{
    loginf;

    traced_assert(object_parser_box_);

    if (object_parser_box_->currentIndex() >= 0)
    {
        unsigned int cat = object_parser_box_->currentText().toUInt();

        traced_assert(task_.schema() != nullptr);
        std::shared_ptr<ASTERIXJSONParsingSchema> current = task_.schema();

        traced_assert(current->hasObjectParser(cat));
        current->removeParser(cat);

        updateParserBox();
        selectedObjectParserSlot(object_parser_box_->currentText());
    }
}

void ASTERIXImportTaskWidget::selectedObjectParserSlot(const QString& text)
{
    loginf << "text '" << text.toStdString()
           << "'";

    traced_assert(object_parser_widget_);

    if (!text.size())
    {
        while (object_parser_widget_->count() > 0)  // remove all widgets
        {
            auto w = object_parser_widget_->widget(0);
            object_parser_widget_->removeWidget(w);
            w->deleteLater();
        }
        object_parser_widgets_.clear();
        return;
    }

    traced_assert(text.size());

    traced_assert(object_parser_box_);
    unsigned int cat = text.toUInt();

    traced_assert(task_.schema() != nullptr);
    traced_assert(task_.schema()->hasObjectParser(cat));

    auto id = text.toStdString();

    if (object_parser_widgets_.count(id) == 0)
    {
        auto w = task_.schema()->parser(cat).createWidget();
        object_parser_widget_->addWidget(w);
        object_parser_widgets_[ id ] = w;
    }

    auto w = object_parser_widgets_.at(id);

    object_parser_widget_->setCurrentWidget(w);
}

void ASTERIXImportTaskWidget::fileLineIDEditSlot(const QString& text)
{
    bool ok;

    unsigned int line_id = text.toUInt(&ok);

    traced_assert(ok);

    traced_assert(line_id > 0 && line_id <= 4);

    loginf << "value '" << text.toStdString()
           << "' line id " << line_id;

    task_.settings().file_line_id_ = line_id-1; // from 1...4
}

void ASTERIXImportTaskWidget::dateChangedSlot(QDate date)
{
    string tmp = date.toString("yyyy-MM-dd").toStdString();

    loginf << "start" << tmp;

    task_.settings().date_ = Time::fromDateString(tmp);
}

void ASTERIXImportTaskWidget::updateParserBox()
{
    loginf;

    traced_assert(object_parser_box_);
    object_parser_box_->clear();

    if (task_.schema() != nullptr)
    {
        for (auto& parser_it : *task_.schema())  // over all object parsers
        {
            object_parser_box_->addItem(QString::number(parser_it.first));
        }
    }
}

void ASTERIXImportTaskWidget::resetDateChangedSlot()
{
    QCheckBox* box = dynamic_cast<QCheckBox*>(sender());
    traced_assert(box);

    task_.settings().reset_date_between_files_ = box->checkState() == Qt::Checked;
}

void ASTERIXImportTaskWidget::ignoreTimeJumpsCheckedSlot()
{
    loginf;
    traced_assert(ignore_timejumps_check_);

    task_.settings().ignore_time_jumps_ = ignore_timejumps_check_->checkState() == Qt::Checked;
}

void ASTERIXImportTaskWidget::debugChangedSlot()
{
    QCheckBox* box = dynamic_cast<QCheckBox*>(sender());
    traced_assert(box);

    task_.settings().debug_jasterix_ = box->checkState() == Qt::Checked;
}

void ASTERIXImportTaskWidget::updateSourcesGrid()
{
    QLayoutItem* child;
    while (!sources_grid_->isEmpty() && (child = sources_grid_->takeAt(0)) != nullptr)
    {
        if (child->widget())
            delete child->widget();
        delete child;
    }

    if (task_.source().isNetworkType())
    {
        sources_grid_->addWidget(new QLabel("Source: Network"), 0, 0);
    }
    else // files
    {
        QStringList headers;
        headers << "";
        headers << "Name";
        headers << "Decoding";
        headers << "Error";

        QTreeWidget* tree_widget = new QTreeWidget;
        tree_widget->setColumnCount(headers.count());
        tree_widget->setHeaderLabels(headers);

        tree_widget->header()->setSectionResizeMode(0, QHeaderView::ResizeMode::ResizeToContents);
        tree_widget->header()->setSectionResizeMode(1, QHeaderView::ResizeMode::ResizeToContents);
        tree_widget->header()->setSectionResizeMode(2, QHeaderView::ResizeMode::ResizeToContents);
        tree_widget->header()->setSectionResizeMode(3, QHeaderView::ResizeMode::Stretch);

        unsigned int file_idx = 0;

        for (const auto& file_info : task_.source().files())
        {
            std::string name   = file_info.filename;
            std::string status = file_info.decodingTested() ? (!file_info.canDecode() ? "Error" : "OK") : "?";
            std::string descr  = file_info.error.errinfo;

            auto item = new QTreeWidgetItem;
            item->setCheckState(0, file_info.used ? Qt::CheckState::Checked : Qt::CheckState::Unchecked);
            item->setText(1, QString::fromStdString(name  ));
            item->setText(2, QString::fromStdString(status));
            item->setText(3, QString::fromStdString(descr ));

            item->setData(0, Qt::UserRole, QVariant(QPoint(file_idx, -1)));

            bool has_error = file_info.decodingTested() && !file_info.canDecode();

            if (has_error)
                item->setFlags(Qt::ItemIsSelectable);

            //file itself has no error? => add sections
            if (!has_error)
            {
                unsigned int section_idx = 0;

                for (const auto& section : file_info.sections)
                {
                    std::string name   = section.description;
                    std::string status = file_info.decodingTested() ? (section.error.hasError() ? "Error" : "OK") : "?";
                    std::string descr  = section.error.errinfo;

                    auto sec_item = new QTreeWidgetItem;
                    sec_item->setCheckState(0, section.used ? Qt::CheckState::Checked : Qt::CheckState::Unchecked);
                    sec_item->setText(1, QString::fromStdString(name  ));
                    sec_item->setText(2, QString::fromStdString(status));
                    sec_item->setText(3, QString::fromStdString(descr ));

                    sec_item->setData(0, Qt::UserRole, QVariant(QPoint(file_idx, section_idx)));

                    bool has_error = file_info.decodingTested() && section.error.hasError();

                    if (has_error)
                        sec_item->setFlags(Qt::ItemIsSelectable);

                    item->addChild(sec_item);

                    ++section_idx;
                }
            }

            tree_widget->addTopLevelItem(item);

            ++file_idx;
        }

        tree_widget->expandAll();

        connect(tree_widget, &QTreeWidget::itemClicked, this, &ASTERIXImportTaskWidget::sourceClicked);

        sources_grid_->addWidget(tree_widget, 0, 0);
    }
}

ASTERIXOverrideWidget* ASTERIXImportTaskWidget::overrideWidget() const
{
    return override_widget_;
}

void ASTERIXImportTaskWidget::decodingStateChangedSlot()
{
    updateSourcesGrid();
}

void ASTERIXImportTaskWidget::sourceClicked(QTreeWidgetItem* item, int column)
{
    if (item && column == 0)
    {
        bool selected = item->checkState(0) == Qt::CheckState::Checked;

        QPoint index = item->data(0, Qt::UserRole).toPoint();
        task_.source().setFileUsage(selected, (size_t)index.x(), index.y());
    }
}
