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

#include "fileimportdialog.h"

#include "files.h"

#include "geotiff.h"

#include <QPushButton>
#include <QCheckBox>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTreeWidget>

#include <iostream>

/***************************************************************************
 * FileImportDialog
 ***************************************************************************/

/**
*/
FileImportDialog::FileImportDialog(const std::vector<std::string>& files,
                                   const std::string& import_type_name,
                                   QWidget* parent, 
                                   Qt::WindowFlags f)
:   QDialog(parent, f)
,   files_ (files)
{
    QString importer_name = QString::fromStdString(import_type_name);

    setWindowTitle("Import " + (importer_name.isEmpty() ? "Files": importer_name));

    auto layout = new QVBoxLayout;
    setLayout(layout);

    tree_widget_ = new QTreeWidget(this);

    layout->addWidget(tree_widget_);

    custom_layout_ = new QVBoxLayout;
    layout->addLayout(custom_layout_);

    //layout->addStretch(1);

    auto button_layout = new QHBoxLayout;
    layout->addLayout(button_layout);

    ok_button_         = new QPushButton("Import");
    auto cancel_button = new QPushButton("Cancel");

    button_layout->addWidget(cancel_button);
    button_layout->addStretch(1);
    button_layout->addWidget(ok_button_);
    
    connect(tree_widget_, &QTreeWidget::itemChanged, this, &FileImportDialog::itemChanged);
    connect(ok_button_, &QPushButton::pressed, this, &QDialog::accept);
    connect(cancel_button, &QPushButton::pressed, this, &QDialog::reject);
}

/**
*/
void FileImportDialog::showEvent(QShowEvent *event)
{
    init();
}

/**
*/
void FileImportDialog::init()
{
    if (init_)
        return;

    init_ = true;

    QStringList headers_default;
    headers_default << "Import";
    headers_default << "Filename";
    headers_default << infoColumnName();

    num_default_cols_ = headers_default.count();

    QStringList headers = headers_default;

    auto custom_columns = customColumns();

    for (const auto& c : custom_columns)
        headers << QString::fromStdString(c.name);

    tree_widget_->setColumnCount(headers.count());
    tree_widget_->setHeaderLabels(headers);

    tree_widget_->header()->setSectionResizeMode(0, QHeaderView::ResizeMode::Fixed);
    tree_widget_->header()->setSectionResizeMode(1, QHeaderView::ResizeMode::ResizeToContents);
    tree_widget_->header()->setSectionResizeMode(2, QHeaderView::ResizeMode::ResizeToContents);

    int idx = 0;
    for (const auto& c : custom_columns)
        tree_widget_->header()->setSectionResizeMode(num_default_cols_ + idx++, c.resize_mode);

    idx = 0;
    for (const auto& f : files_)
    {
        if (f.empty() || item_map_.find(f) != item_map_.end())
            continue;

        auto item = new QTreeWidgetItem;
        tree_widget_->addTopLevelItem(item);

        item_map_[ f ] = idx;

        initItem(item, idx, f);

        ++idx;
    }

    //tree_widget_->header()->resizeSections();

    checkImportOk();
}

/**
 */
void FileImportDialog::initItem(QTreeWidgetItem* item, int idx, const std::string& fn)
{
    auto file_ok = checkFile(fn);

    item->setCheckState(0, file_ok.first ? Qt::Checked : Qt::Unchecked);
    item->setText(1, QString::fromStdString(fn));
    item->setText(2, QString::fromStdString(file_ok.second));

    initItem_impl(item, idx, fn);

    if (!file_ok.first)
        item->setFlags(item->flags() & ~Qt::ItemFlag::ItemIsEnabled);
}

/**
 */
int FileImportDialog::fileItemIndex(const std::string& fn) const
{
    auto it = item_map_.find(fn);
    if (it == item_map_.end())
        return -1;

    return it->second;
}

/**
 */
QTreeWidgetItem* FileImportDialog::fileItem(const std::string& fn) const
{
    int idx = fileItemIndex(fn);
    if (idx < 0 || idx >= tree_widget_->topLevelItemCount())
        return nullptr;

    return tree_widget_->topLevelItem(idx);
}

/**
*/
std::pair<bool, std::string> FileImportDialog::checkFile(const std::string& fn) const
{
    if (!Utils::Files::fileExists(fn))
        return std::make_pair(false, "File does not exist");

    auto ret = checkFile_impl(fn);
    if (!ret.first)
    {
        if (ret.second.empty())
            ret.second = "Unknown error";
        return ret;
    }

    return std::make_pair(true, ret.second);
}

/**
*/
std::pair<bool, std::string> FileImportDialog::checkFile_impl(const std::string& fn) const
{
    return std::make_pair(true, "");
}

/**
*/
bool FileImportDialog::importFile(const std::string& fn) const
{
    auto item = fileItem(fn);
    if (!item)
        return false;

    return item->checkState(0) == Qt::Checked;
}

/**
*/
bool FileImportDialog::importOk() const
{
    for (const auto& f : files_)
        if (importFile(f))
            return true;

    return false;
}

/**
*/
void FileImportDialog::checkImportOk()
{
    bool ok = importOk();

    ok_button_->setEnabled(ok);
}

/**
*/
void FileImportDialog::itemChanged(QTreeWidgetItem* item, int column)
{
    itemChanged_impl(item, column);

    if (column == 0)
    {
        checkImportOk();
    }
}

/**
*/
QString FileImportDialog::infoColumnName() const
{
    return "Information";
}

/***************************************************************************
 * GeoTIFFImportDialog
 ***************************************************************************/

/**
*/
GeoTIFFImportDialog::GeoTIFFImportDialog(const std::vector<std::string>& files,
                                         QWidget* parent, 
                                         Qt::WindowFlags f)
:   FileImportDialog(files, "GeoTIFF", parent, f)
{
}

/**
*/
std::pair<bool, std::string> GeoTIFFImportDialog::checkFile_impl(const std::string& fn) const
{
    auto info = GeoTIFF::getInfo(fn);

    bool gtiff_ok = info.isValid();

    std::string msg = gtiff_ok ? std::to_string(info.img_w) + "x" + std::to_string(info.img_h) + " - " + std::to_string(info.bands) + " band(s)": 
                                 "Invalid GeoTIFF (Code " + std::to_string((int)info.error) + ")";
    
    return std::make_pair(gtiff_ok, msg);
}

QString GeoTIFFImportDialog::infoColumnName() const
{
    return "Image Size";
}

/**
*/
void GeoTIFFImportDialog::itemChanged_impl(QTreeWidgetItem* item, int column)
{
}
