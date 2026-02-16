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

#include "json.hpp"

#include <QDialog>

class QGridLayout;

namespace dbContent
{

class LabelGenerator;

class LabelContentDialog : public QDialog
{
    Q_OBJECT

signals:
    void doneSignal();

public slots:
    void selectedVarChangedSlot();
    void doneClickedSlot();

public:
    LabelContentDialog(const std::string& dbcontent_name, LabelGenerator& label_generator);

    nlohmann::json labelConfig() const;

protected:
    std::string dbcontent_name_;
    LabelGenerator& label_generator_;

    nlohmann::json label_config_;

    QGridLayout* var_grid_{nullptr};

    QPushButton* done_button_{nullptr};

    void createVariableGrid();
};

}

