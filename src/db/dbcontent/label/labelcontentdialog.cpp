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

#include "labelcontentdialog.h"
#include "dbcontent/label/labelgenerator.h"
#include "dbcontent/variable/variableselectionwidget.h"
#include "dbcontent/variable/reportvariable.h"
#include "logger.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontent.h"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

using namespace std;
using namespace nlohmann;

namespace dbContent
{

LabelContentDialog::LabelContentDialog(const std::string& dbcontent_name, LabelGenerator& label_generator)
    : dbcontent_name_(dbcontent_name), label_generator_(label_generator),
      label_config_(label_generator_.labelConfig())
{
    setWindowTitle(("Edit "+dbcontent_name_+" Label Content").c_str());
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

    setModal(true);
    setObjectName("label_content_dialog");

    setMinimumSize(QSize(700, 200));

    QVBoxLayout* main_layout = new QVBoxLayout();

    var_grid_ = new QGridLayout();
    createVariableGrid();
    main_layout->addLayout(var_grid_);


    QHBoxLayout* button_layout = new QHBoxLayout();

    done_button_ = new QPushButton("Done");
    done_button_->setObjectName("done_button");
    connect(done_button_, &QPushButton::clicked, this, &LabelContentDialog::doneClickedSlot);
    button_layout->addWidget(done_button_);

    main_layout->addLayout(button_layout);

    setLayout(main_layout);
}

nlohmann::json LabelContentDialog::labelConfig() const
{
    return label_config_;
}

void LabelContentDialog::selectedVarChangedSlot()
{
    VariableSelectionWidget* var_widget = dynamic_cast<VariableSelectionWidget*>(sender());
    traced_assert(var_widget);

    unsigned int key = var_widget->property("key").toUInt();

    loginf << "key " << key;

    traced_assert(label_config_.contains(dbcontent_name_));
    json& dbcont_def = label_config_.at(dbcontent_name_);

    if (var_widget->hasVariable())
    {
        Variable& var = var_widget->selectedVariable();

        dbcont_def[to_string(key)] = var.name();
    }
    else if (var_widget->hasReportVariable())
    {
        // the name of a Report Variable carries its report, "<report>: <display name>"
        dbcont_def[to_string(key)] = var_widget->selectedReportVariable().name();
    }
    else // unselect
    {
        if (dbcont_def.count(to_string(key)))
            dbcont_def.erase(to_string(key));
    }
}

void LabelContentDialog::doneClickedSlot()
{
    emit doneSignal();
}

void LabelContentDialog::createVariableGrid()
{
    traced_assert(var_grid_);

    QFont font_bold;
    font_bold.setBold(true);

    traced_assert(label_config_.contains(dbcontent_name_));
    json& dbcont_def = label_config_.at(dbcontent_name_);

    string key;

    DBContentManager& dbcont_man = label_generator_.dbContentManager();
    DBContent& db_content = dbcont_man.dbContent(dbcontent_name_);

    for (unsigned int row=0; row < 3; row++)
    {
        for (unsigned int col=0; col < 3; col++)
        {
            key = to_string(row*3 + col);

            if (row == 0 && col == 0) // best id
            {
                traced_assert(dbcont_def.contains(key));
                string var_name = dbcont_def.at(key);

                QLabel* best_id = new QLabel(var_name.c_str());
                best_id->setFont(font_bold);
                var_grid_->addWidget(best_id, row, col);
            }
            else
            {
                VariableSelectionWidget* var_widget = new VariableSelectionWidget(label_generator_.dbContentManager());
                var_widget->setProperty("key", row*3 + col);
                var_widget->setObjectName(QString("label_var_%1").arg(row*3 + col));
                var_widget->showDBContentOnly(dbcontent_name_);

                if (dbcont_def.contains(key))
                {
                    string var_name = dbcont_def.at(key);

                    if (db_content.hasVariable(var_name))
                        var_widget->selectedVariable(db_content.variable(var_name));
                    else
                    {
                        // a Report Variable, shown if its report is in this database
                        bool found = false;

                        for (const auto& content : dbcont_man.reportContents())
                        {
                            if (content->hasVariable(var_name) &&
                                content->variable(var_name).existsIn(dbcontent_name_))
                            {
                                var_widget->selectedReportVariable(content->variable(var_name));
                                found = true;
                                break;
                            }
                        }

                        if (!found)
                            logwrn << "label variable '" << var_name << "' not available in "
                                   << dbcontent_name_;
                    }
                }

                if (row <= 1 && col <= 1)
                    var_widget->setReadOnly(true); // set static

                connect(var_widget, &VariableSelectionWidget::selectionChanged,
                        this, &LabelContentDialog::selectedVarChangedSlot);

                var_grid_->addWidget(var_widget, 2*row, 2*col);
            }
        }
    }

    // add lines
    {
        QFrame *line_hor = new QFrame(this);
        line_hor->setLineWidth(2);
        line_hor->setMidLineWidth(1);
        line_hor->setFrameShape(QFrame::HLine);
        line_hor->setStyleSheet("background-color:black");

        var_grid_->addWidget(line_hor, 1, 0, 1, 2);
    }

    {
        QFrame *line_vert = new QFrame(this);
        line_vert->setLineWidth(2);
        line_vert->setMidLineWidth(1);
        line_vert->setFrameShape(QFrame::VLine);
        line_vert->setStyleSheet("background-color:black");

        var_grid_->addWidget(line_vert, 0, 1, 2, 1);
    }

    {
        QFrame *line_hor = new QFrame(this);
        line_hor->setLineWidth(2);
        line_hor->setMidLineWidth(1);
        line_hor->setFrameShape(QFrame::HLine);
        line_hor->setStyleSheet("background-color:black");

        var_grid_->addWidget(line_hor, 3, 0, 1, 4);
    }

    {
        QFrame *line_vert = new QFrame(this);
        line_vert->setLineWidth(2);
        line_vert->setMidLineWidth(1);
        line_vert->setFrameShape(QFrame::VLine);
        line_vert->setStyleSheet("background-color:black");

        var_grid_->addWidget(line_vert, 0, 3, 4, 1);
    }


}

}

