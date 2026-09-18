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

#include "asteriximportprogressdialog.h"
#include "asteriximportsource.h"
#include "asterixdecoderbase.h"
#include "stringconv.h"
#include "logger.h"

#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include <cmath>

namespace
{
    enum Column
    {
        ColumnFilename = 0,
        ColumnSize,
        ColumnStatus
    };

    /**
     * Returns the value as an unsigned number, or 0 if it is not usable.
     * The rates are undefined during the first update, when no time has passed yet.
     */
    unsigned int finiteValue(float v)
    {
        return (std::isfinite(v) && v > 0) ? (unsigned int)v : 0;
    }
}

/**
*/
ASTERIXImportProgressDialog::ASTERIXImportProgressDialog(const ASTERIXImportSource& source,
                                                         QWidget* parent)
:   QDialog (parent)
,   source_ (source)
{
    setWindowTitle("Importing ASTERIX Recording(s)");

    // the dialog width is fixed by the layout, so the content cannot resize the window.
    // modality makes the window manager re-assert focus whenever the geometry changes,
    // which used to happen on every progress update
    setMinimumWidth(DialogMinWidth);

    createUI();
    createFileRows();
    updateTreeHeight();
}

/**
*/
ASTERIXImportProgressDialog::~ASTERIXImportProgressDialog() = default;

/**
*/
void ASTERIXImportProgressDialog::createUI()
{
    QVBoxLayout* main_layout = new QVBoxLayout;

    message_label_ = new QLabel("Decoding ASTERIX Data...");
    main_layout->addWidget(message_label_);

    file_tree_ = new QTreeWidget;
    file_tree_->setColumnCount(3);

    QStringList headers;
    headers << "Filename";
    headers << "Size (MB)";
    headers << "Status";
    file_tree_->setHeaderLabels(headers);

    file_tree_->header()->setSectionResizeMode(ColumnFilename, QHeaderView::Stretch);
    file_tree_->header()->setSectionResizeMode(ColumnSize,     QHeaderView::ResizeToContents);
    file_tree_->header()->setSectionResizeMode(ColumnStatus,   QHeaderView::ResizeToContents);
    file_tree_->header()->setStretchLastSection(false);

    file_tree_->setRootIsDecorated(false);
    file_tree_->setUniformRowHeights(true);
    file_tree_->setSelectionMode(QAbstractItemView::NoSelection);
    file_tree_->setFocusPolicy(Qt::NoFocus);
    // long recording paths are cut at the front, so the recording name stays readable
    file_tree_->setTextElideMode(Qt::ElideLeft);

    main_layout->addWidget(file_tree_);

    progress_bar_ = new QProgressBar;
    progress_bar_->setRange(0, 100);
    progress_bar_->setValue(0);

    main_layout->addWidget(progress_bar_);

    QHBoxLayout* time_layout = new QHBoxLayout;

    elapsed_label_ = new QLabel;
    time_layout->addWidget(elapsed_label_);
    time_layout->addStretch(1);

    remaining_label_ = new QLabel;
    time_layout->addWidget(remaining_label_);

    main_layout->addLayout(time_layout);

    QHBoxLayout* records_layout = new QHBoxLayout;
    records_layout->addStretch(1);

    records_label_ = new QLabel;
    records_layout->addWidget(records_label_);

    main_layout->addLayout(records_layout);

    main_layout->addStretch(1);

    QHBoxLayout* button_layout = new QHBoxLayout;
    button_layout->addStretch(1);

    abort_button_ = new QPushButton("Abort");
    abort_button_->setIcon(QIcon());
    abort_button_->setToolTip("Stop the import and discard the data decoded so far");
    connect(abort_button_, &QPushButton::clicked, this, &ASTERIXImportProgressDialog::abortSlot);

    button_layout->addWidget(abort_button_);

    main_layout->addLayout(button_layout);

    setLayout(main_layout);
}

/**
*/
void ASTERIXImportProgressDialog::createFileRows()
{
    const auto& file_infos = source_.files();

    for (std::size_t idx = 0; idx < file_infos.size(); ++idx)
    {
        const auto& file_info = file_infos.at(idx);

        // skip unused files
        if (!file_info.used)
            continue;

        QTreeWidgetItem* item = new QTreeWidgetItem;

        item->setText(ColumnFilename, QString::fromStdString(file_info.filename));
        item->setToolTip(ColumnFilename, QString::fromStdString(file_info.filename));

        const double mb = file_info.sizeInBytes(/*used_only=*/true) / (1024.0 * 1024.0);

        item->setText(ColumnSize, QString::number(mb, 'f', 2));
        item->setTextAlignment(ColumnSize, Qt::AlignRight | Qt::AlignVCenter);

        item->setText(ColumnStatus, "Pending");

        file_tree_->addTopLevelItem(item);

        file_items_.push_back(item);
        file_indices_.push_back(idx);
    }
}

/**
*/
void ASTERIXImportProgressDialog::updateTreeHeight()
{
    const int num_rows = (int)file_items_.size();

    const int row_height = num_rows ? file_tree_->sizeHintForRow(0)
                                    : file_tree_->fontMetrics().height() + 6;

    // show up to MaxVisibleFileRows files, scroll beyond that, so a long file list
    // cannot grow the dialog past the screen height
    const int visible_rows = std::max(std::min(num_rows, MaxVisibleFileRows), MinVisibleFileRows);

    file_tree_->setMinimumHeight(file_tree_->header()->sizeHint().height()
                                 + visible_rows * row_height
                                 + 2 * file_tree_->frameWidth());
}

/**
*/
void ASTERIXImportProgressDialog::updateStatus(const ASTERIXDecodeStatus& status)
{
    const auto& file_infos = source_.files();

    for (std::size_t row = 0; row < file_items_.size(); ++row)
    {
        const auto&      file_info = file_infos.at(file_indices_.at(row));
        QTreeWidgetItem* item      = file_items_.at(row);

        const bool current = (file_info.filename == status.current_filename);

        QString status_text;

        if (current)
            status_text = "Decoding";
        else if (file_info.fileProcessed())
            status_text = "Done";
        else
            status_text = "Pending";

        if (file_info.hasError())
        {
            status_text += " (errors detected)";
            item->setForeground(ColumnStatus, QBrush(Qt::red));
        }

        // only touch the cells that changed, a repaint on every update makes the list flicker
        if (item->text(ColumnStatus) != status_text)
            item->setText(ColumnStatus, status_text);

        QFont font = item->font(ColumnFilename);

        if (font.bold() != current)
        {
            font.setBold(current);

            item->setFont(ColumnFilename, font);
            item->setFont(ColumnSize,     font);
            item->setFont(ColumnStatus,   font);
        }

        if (current && (int)row != current_row_)
        {
            current_row_ = (int)row;
            file_tree_->scrollToItem(item);
        }
    }

    int progress = (int)std::lround(std::isfinite(status.progress) ? status.progress : 0.0f);
    progress     = std::max(0, std::min(100, progress));

    progress_bar_->setValue(progress);

    elapsed_label_->setText(
        ("Elapsed: " + Utils::String::timeStringFromDouble(status.elapsed_seconds, false)).c_str());
    remaining_label_->setText(
        ("Remaining: " + Utils::String::timeStringFromDouble(status.remaining_seconds, false)).c_str());
    records_label_->setText(
        QString("Records per Second: %1").arg(finiteValue(status.records_per_second)));
}

/**
*/
void ASTERIXImportProgressDialog::setMessage(const QString& message)
{
    message_label_->setText(message);
}

/**
*/
void ASTERIXImportProgressDialog::abortSlot()
{
    if (aborted_)
        return;

    loginf << "import aborted by user";

    aborted_ = true;

    abort_button_->setEnabled(false);

    // the import task stops on the next update, the dialog disappears at once,
    // as the previous progress dialog did
    hide();
}

/**
*/
void ASTERIXImportProgressDialog::reject()
{
    abortSlot();
}
