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

#pragma once

#include <string>
#include <map>

#include <QDialog>
#include <QHeaderView>

class QVBoxLayout;
class QPushButton;
class QCheckBox;
class QTreeWidget;
class QTreeWidgetItem;

/**
 */
class FileImportDialog : public QDialog
{
public:
    struct CustomColumn
    {
        CustomColumn(const std::string& col_name,
                     bool col_checkable,
                     QHeaderView::ResizeMode col_resize_mode)
        :   name       (col_name)
        ,   checkable  (col_checkable)
        ,   resize_mode(col_resize_mode) {} 

        std::string             name;
        bool                    checkable = false;
        QHeaderView::ResizeMode resize_mode = QHeaderView::ResizeMode::ResizeToContents;
    };

    FileImportDialog(const std::vector<std::string>& files,
                     const std::string& import_type_name = "",
                     QWidget* parent = nullptr, 
                     Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~FileImportDialog() = default;

    void init();

    virtual bool importOk() const;

    bool importFile(const std::string& fn) const;

protected:
    virtual void showEvent(QShowEvent *event) override;

    virtual std::vector<CustomColumn> customColumns() const { return {}; }
    virtual std::pair<bool, std::string> checkFile_impl(const std::string& fn) const;
    virtual void initItem_impl(QTreeWidgetItem* item, int idx, const std::string& fn) {}
    virtual void itemChanged_impl(QTreeWidgetItem* item, int column) {}
    virtual void init_impl() {}

    virtual QString infoColumnName() const;

    int fileItemIndex(const std::string& fn) const;
    QTreeWidgetItem* fileItem(const std::string& fn) const;

    QTreeWidget* treeWidget() { return tree_widget_; }
    const QTreeWidget* treeWidget() const { return tree_widget_; }
    QVBoxLayout* customLayout() { return custom_layout_; }
    int numDefaultColumns() const { return num_default_cols_; }

private:
    void itemChanged(QTreeWidgetItem* item, int column);
    std::pair<bool, std::string> checkFile(const std::string& fn) const;
    void initItem(QTreeWidgetItem* item, int idx, const std::string& fn);

    void checkImportOk();

    std::vector<std::string>    files_;
    std::map<std::string, int>  item_map_;

    bool init_             = false;
    int  num_default_cols_ = 0;

    QTreeWidget* tree_widget_   = nullptr;
    QPushButton* ok_button_     = nullptr;
    QVBoxLayout* custom_layout_ = nullptr;
};

/**
*/
class GeoTIFFImportDialog : public FileImportDialog
{
public:
    GeoTIFFImportDialog(const std::vector<std::string>& files,
                        QWidget* parent = nullptr, 
                        Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~GeoTIFFImportDialog() = default;

protected:
    virtual std::pair<bool, std::string> checkFile_impl(const std::string& fn) const override;
    virtual void itemChanged_impl(QTreeWidgetItem* item, int column) override;

    virtual QString infoColumnName() const override;
};
