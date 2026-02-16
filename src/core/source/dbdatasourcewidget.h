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

#include <QWidget>

#include <map>
#include <functional>

class QCheckBox;
class QLabel;
class QPushButton;
class QGridLayout;
class QVBoxLayout;

class DataSourceManager;

namespace dbContent
{

class DBDataSource;

class DBDataSourceWidget : public QWidget
{
    Q_OBJECT

  public slots:
    void loadingChangedSlot();

    void lineButtonClickedSlot();

  public:
    explicit DBDataSourceWidget(
        DBDataSource& src,
        std::function<bool()> get_use_ds_func,
        std::function<void(bool)> set_use_ds_func,
        std::function<bool(unsigned int)> get_use_ds_line_func,
        std::function<void(unsigned int, bool)> set_use_ds_line_func,
        std::function<bool()> show_counts_func,
        QWidget *parent = nullptr);

    void setLoadChecked (bool value);
    void updateContent();

    unsigned int getLabelMinWidth();
    void updateLabelMinWidth(unsigned int width);

  protected:
    DBDataSource& src_;
    DataSourceManager& ds_man_;

    std::function<bool()> get_use_ds_func_;
    std::function<void(bool)> set_use_ds_func_;
    std::function<bool(unsigned int)> get_use_ds_line_func_;
    std::function<void(unsigned int, bool)> set_use_ds_line_func_;
    std::function<bool()> show_counts_func_;

    QVBoxLayout* main_layout_ {nullptr};
    QGridLayout* grid_layout_ {nullptr};

    QCheckBox* load_check_ {nullptr};

    std::map<std::string, QPushButton*> line_buttons_; // LX -> button, has all 4 lines, can hide
    std::map<std::string, QLabel*> content_labels_; // cont -> label
    std::map<std::string, QLabel*> loaded_cnt_labels_; // cont -> loaded count label
    std::map<std::string, QLabel*> total_cnt_labels_; // cont -> total count label

            //bool last_net_lines_shown_ {false};
    bool last_show_counts_ {false}; // indicates if counts where added to layout last time

    bool needsRecreate();
    void recreateWidgets();
    QWidget* createLinesWidget();
    void updateWidgets(); // updates all widgets to be stored in layout
};

}
