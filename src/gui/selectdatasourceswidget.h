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

#include <QFrame>

class QCheckBox;
class QGridLayout;

class SelectDataSourcesWidget : public QFrame
{
    Q_OBJECT

signals:
    void selectionChangedSignal(std::map<std::string, bool> selection);

protected slots:
    void toggleDataSourceSlot();

public:
    SelectDataSourcesWidget(const std::string& title, const std::string& ds_type,
                            QWidget* parent=nullptr, Qt::WindowFlags f=Qt::WindowFlags());

    virtual ~SelectDataSourcesWidget();

    void updateSelected(std::map<std::string, bool> selection);

protected:
    std::string title_;
    std::string ds_type_;

    QGridLayout* data_source_layout_ {nullptr};
    std::map<unsigned int, QCheckBox*> data_sources_checkboxes_;

    void updateCheckboxesChecked();

};
