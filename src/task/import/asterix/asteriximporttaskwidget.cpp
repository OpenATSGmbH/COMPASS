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
#include "asterixframingcombobox.h"
#include "asteriximportdatasourceswidget.h"
#include "asteriximporttask.h"
#include "compass.h"
#include "asterixoverridewidget.h"
#include "db_context_manager.h"
#include "logger.h"
#include "dbcontent/selectdialog.h"
#include "util/timeconv.h"
#include "util/files.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QInputDialog>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QApplication>
#include <QProgressDialog>
#include <QPushButton>
#include <QSignalBlocker>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QDateEdit>
#include <QTreeWidgetItem>
#include <QTreeWidget>
#include <QDesktopServices>
#include <QHeaderView>
#include <QUrl>

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
    addDataSourcesTab();
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

        //main_tab_layout->addStretch();

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

    //main_tab_layout->addStretch();

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

    QWidget* decoder_tab = new QWidget();
    QVBoxLayout* decoder_layout = new QVBoxLayout();

    // framing controls (import-specific)
    {
        QGridLayout* framing_grid = new QGridLayout();

        QLabel* framing_label = new QLabel("Framing");
        framing_grid->addWidget(framing_label, 0, 0);

        framing_combo_ = new ASTERIXFramingComboBox(task_);
        connect(framing_combo_, &ASTERIXFramingComboBox::changedFraming,
                this, &ASTERIXImportTaskWidget::framingChangedSlot);
        framing_grid->addWidget(framing_combo_, 0, 1);

        framing_edit_ = new QPushButton("Edit");
        connect(framing_edit_, &QPushButton::clicked,
                this, &ASTERIXImportTaskWidget::framingEditSlot);
        framing_grid->addWidget(framing_edit_, 0, 2);

        updateFramingControls();

        decoder_layout->addLayout(framing_grid);
    }

    // ASTERIX category config (editions from DBContext, decode flags from import task)
    config_widget_ = new ASTERIXConfigWidget(
        task_.compass().dbContextManager(),
        [this](unsigned int cat) { return task_.decodeCategory(cat); },
        [this](unsigned int cat, bool decode) { task_.decodeCategory(cat, decode); },
        this);
    // re-probe the file decoding whenever a category is enabled/disabled or an
    // edition / REF / SPF is changed
    connect(config_widget_, &ASTERIXConfigWidget::decodingConfigChangedSignal,
            this, [this]() { task_.testFileDecoding(); });
    decoder_layout->addWidget(config_widget_);

    decoder_tab->setLayout(decoder_layout);
    tab_widget_->addTab(decoder_tab, "Decoder");
}

void ASTERIXImportTaskWidget::addOverrideTab()
{
    traced_assert(tab_widget_);

    override_widget_ = new ASTERIXOverrideWidget(task_, this);
    tab_widget_->addTab(override_widget_, "Override/Filter");
}

void ASTERIXImportTaskWidget::addDataSourcesTab()
{
    traced_assert(tab_widget_);

    data_sources_widget_ = new ASTERIXImportDataSourcesWidget(task_);
    data_sources_tab_index_ = tab_widget_->addTab(data_sources_widget_, "Data Sources");

    connect(data_sources_widget_, &ASTERIXImportDataSourcesWidget::warningsChanged,
            this, &ASTERIXImportTaskWidget::dataSourcesWarningsChangedSlot);

    // initial state: widget already ran rebuildAll() in its constructor, so the
    // very first signal was emitted before we connected. Seed the icon now.
    dataSourcesWarningsChangedSlot(data_sources_widget_->hasWarnings());
}

void ASTERIXImportTaskWidget::dataSourcesWarningsChangedSlot(bool any)
{
    if (data_sources_tab_index_ < 0 || !tab_widget_)
        return;

    if (any)
        tab_widget_->setTabIcon(data_sources_tab_index_,
                                Utils::Files::IconProvider::getIcon("hint.png"));
    else
        tab_widget_->setTabIcon(data_sources_tab_index_, QIcon());
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
    add_object_parser_button_->setEnabled(task_.compass().expertMode());
    parser_manage_layout->addWidget(add_object_parser_button_);

    delete_object_parser_button_ = new QPushButton("Remove");
    connect(delete_object_parser_button_, SIGNAL(clicked()), this, SLOT(removeObjectParserSlot()));
    delete_object_parser_button_->setEnabled(task_.compass().expertMode());
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
                              "No current JSON Parsing Schema is selected.", QMessageBox::Ok, this);

        m_warning.exec();
        return;
    }

    dbContent::SelectDBContentDialog dialog(task_.compass().dbContentManager());

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
                                  "ASTERIX parser for category already defined.", QMessageBox::Ok, this);

            m_warning.exec();
            return;
        }

        std::string instance = "ASTERIXJSONParserCAT" + to_string(cat) + "0";

        auto& child_json = current->addNewSubConfiguration("ASTERIXJSONParser", instance);
        child_json[Configuration::ParameterSection]["category"] = cat;
        child_json[Configuration::ParameterSection]["dbcontent_name"] = dbcontent_name;

        current->generateSubConfigurable(child_json);
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

    {
        const bool is_network = task_.source().isNetworkType();

        QStringList headers;
        headers << "";
        headers << "Name";
        headers << "Decoding";

        QTreeWidget* tree_widget = new QTreeWidget;
        tree_widget->setColumnCount(headers.count());
        tree_widget->setHeaderLabels(headers);

        tree_widget->header()->setSectionResizeMode(0, QHeaderView::ResizeMode::ResizeToContents);
        tree_widget->header()->setSectionResizeMode(1, QHeaderView::ResizeMode::Stretch);
        tree_widget->header()->setSectionResizeMode(2, QHeaderView::ResizeMode::Stretch);
        tree_widget->header()->setStretchLastSection(false);

        auto statusString = [](bool tested, bool has_error, const std::string& errinfo,
                               bool has_warning, const std::string& warninfo) -> std::string
        {
            if (!tested)     return "?";
            if (has_error)   return errinfo.empty()  ? "Error"   : "Error: "   + errinfo;
            if (has_warning) return warninfo.empty() ? "Warning" : "Warning: " + warninfo;
            return "OK";
        };

        auto buildTooltip = [](const std::string& info,
                               const std::map<unsigned int, size_t>& rpc) -> QString
        {
            QString tip;
            if (!info.empty())
                tip += QString::fromStdString(info);
            if (!rpc.empty())
            {
                if (!tip.isEmpty())
                    tip += "\n\n";
                size_t total = 0;
                for (const auto& kv : rpc)
                {
                    tip += QString("CAT%1: %2\n")
                               .arg(kv.first, 3, 10, QChar('0'))
                               .arg(kv.second);
                    total += kv.second;
                }
                tip += QString("\nTotal: %1 records").arg(total);
            }
            return tip;
        };

        auto addCategoriesChild = [](QTreeWidgetItem* parent, const std::string& contentinfo)
        {
            if (contentinfo.empty())
                return;
            auto* dbc_item = new QTreeWidgetItem;
            dbc_item->setText(1, QString::fromStdString(contentinfo));
            dbc_item->setFlags(Qt::ItemIsEnabled);
            parent->addChild(dbc_item);
        };

        unsigned int file_idx = 0;

        for (const auto& file_info : task_.source().files())
        {
            bool tested    = file_info.decodingTested();
            bool has_error = tested && !file_info.canDecode();
            std::string status = statusString(tested, has_error, file_info.error.errinfo,
                                              file_info.hasWarning(), file_info.warning);

            auto item = new QTreeWidgetItem;
            if (!is_network)
                item->setCheckState(0, file_info.used ? Qt::CheckState::Checked : Qt::CheckState::Unchecked);
            item->setText(1, QString::fromStdString(file_info.filename));
            item->setText(2, QString::fromStdString(status));

            item->setData(0, Qt::UserRole, QVariant(QPoint(file_idx, -1)));

            if (has_error)
                item->setFlags(Qt::ItemIsSelectable);
            else if (is_network)
                item->setFlags(Qt::ItemIsEnabled);

            if (!file_info.hasSections())
            {
                // recording: file row owns categories and tooltip
                QString tip = buildTooltip("", file_info.records_per_category);
                if (!tip.isEmpty())
                    item->setToolTip(1, tip);
                addCategoriesChild(item, file_info.contentinfo);
            }

            unsigned int section_idx = 0;

            for (const auto& section : file_info.sections)
            {
                bool sec_err = tested && section.error.hasError();
                std::string sec_status = statusString(tested, sec_err, section.error.errinfo,
                                                      !section.warning.empty(), section.warning);
                // for network sections, !used means no data was received during the probe;
                // surface that in the status column instead of a warning, and disable the row.
                if (is_network && !sec_err && !section.used && !section.info.empty())
                    sec_status = section.info;

                auto sec_item = new QTreeWidgetItem;
                if (!is_network)
                    sec_item->setCheckState(0, section.used ? Qt::CheckState::Checked : Qt::CheckState::Unchecked);
                sec_item->setText(1, QString::fromStdString(section.description));
                sec_item->setText(2, QString::fromStdString(sec_status));

                sec_item->setData(0, Qt::UserRole, QVariant(QPoint(file_idx, section_idx)));

                if (sec_err)
                    sec_item->setFlags(Qt::ItemIsSelectable);
                else if (is_network && !section.used)
                    sec_item->setFlags(Qt::NoItemFlags);
                else if (is_network)
                    sec_item->setFlags(Qt::ItemIsEnabled);

                QString tip = buildTooltip(section.info, section.records_per_category);
                if (!tip.isEmpty())
                    sec_item->setToolTip(1, tip);

                addCategoriesChild(sec_item, section.contentinfo);

                item->addChild(sec_item);

                ++section_idx;
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
    updateFramingControls();
}

void ASTERIXImportTaskWidget::updateFramingControls()
{
    if (!framing_combo_ || !framing_edit_)
        return;

    // Network and PCAP decoders force "raw/netto" (no framing) via requiredASTERIXFraming();
    // when forced, the combo and edit reflect that and are disabled. The persisted
    // current_file_framing_ is preserved in settings - only the override is active.
    const bool forced = task_.requiresFixedFraming();

    QSignalBlocker block(framing_combo_);
    framing_combo_->setFraming(task_.settings().activeFileFraming());

    framing_combo_->setEnabled(!forced);
    framing_edit_->setEnabled(!forced && task_.settings().activeFileFraming() != "");
}

void ASTERIXImportTaskWidget::sourceClicked(QTreeWidgetItem* item, int column)
{
    if (task_.source().isNetworkType())
        return; // network rows have no per-line opt-out

    if (item && column == 0)
    {
        bool selected = item->checkState(0) == Qt::CheckState::Checked;

        QPoint index = item->data(0, Qt::UserRole).toPoint();
        task_.source().setFileUsage(selected, (size_t)index.x(), index.y());
    }
}

void ASTERIXImportTaskWidget::framingChangedSlot()
{
    traced_assert(framing_combo_);
    loginf << framing_combo_->getFraming();

    task_.settings().setActiveFileFraming(framing_combo_->getFraming());

    updateFramingControls();

    task_.testFileDecoding();
}

void ASTERIXImportTaskWidget::framingEditSlot()
{
    std::string framing_path = "file:///" + task_.jASTERIX()->framingsFolderPath() + "/" +
            task_.settings().activeFileFraming() + ".json";
    loginf << "path '" << framing_path << "'";
    QDesktopServices::openUrl(QUrl(framing_path.c_str()));
}



