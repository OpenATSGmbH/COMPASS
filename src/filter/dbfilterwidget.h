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

#include "ui_test_testable.h"

#include <QFrame>
#include <QMenu>

class DBFilter;

class QWidget;
class QCheckBox;
class QGridLayout;
class QPushButton;

/**
 */
class DBFilterWidget : public QFrame, public ui_test::UITestable
{
    Q_OBJECT

private slots:
    void toggleVisible();
    //void toggleAnd(); // not used
    void toggleActive();

    //void possibleSubFilterChange();
    //void reset();
    //void deleteFilter();
    //void filterEditSlot();

protected slots:
    //void showMenuSlot();

// signals:
//     void possibleFilterChange();
    //void filterEdit(DBFilter* filter);
    //void deleteFilterSignal(DBFilter* filter);
    //void filterContentChanged();

public:
    DBFilterWidget(DBFilter& filter);
    virtual ~DBFilterWidget();

    void addChildWidget(QWidget* widget);
    void updateChildWidget();

    virtual void update(void);

    /// re-reads the widget contents from the filter, e.g. to check via ui
    /// commands which edited values were actually stored
    void uiRefresh() override;

    void collapse();
    void expand();

    int columnWidth(int layout_column) const;
    void setFixedColumnWidth(int layout_column, int width);
    void setMaximumColumnWidth(int layout_column, int width);
    void setFlexibleColumnWidth(int layout_column, int width);

protected:
    //void createMenu();
    void deleteChildrenFromLayout();
    void addNameValuePair(const std::string& label, QWidget* widget, int row = -1, int col = 0);
    void addNameValuePair(const std::string& label, const std::string& label2, int row = -1, int col = 0);

    DBFilter& filter_;

    QWidget* child_ {nullptr}; // Child widget from DBFilter

    QCheckBox* visible_checkbox_ {nullptr};
    QCheckBox* active_checkbox_ {nullptr};

    //QPushButton* manage_button_ {nullptr};

    QGridLayout* child_layout_ {nullptr};

    //QMenu menu_;
};
