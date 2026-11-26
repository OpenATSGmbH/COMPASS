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

#include "dbcontent/label/labelgenerator.h"

#include <QWidget>
#include <QIcon>

//class QListWidget;
class QPushButton;
class QGridLayout;

namespace dbContent
{

class LabelGenerator;

class LabelDSWidget : public QWidget
{
    Q_OBJECT

  public slots:
    void updateListSlot();
    void sourceClickedSlot();
    void changeLineSlot();
    void changeDirectionSlot();
    void selectDirectionSlot();
    void selectLineSlot();

public:
    LabelDSWidget(LabelGenerator& label_generator, QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~LabelDSWidget();

    void forceUpdateList();

protected:
    LabelGenerator& label_generator_;

    //QListWidget* list_widget_{nullptr};

    QIcon arrow_lu_;
    QIcon arrow_ru_;
    QIcon arrow_ld_;
    QIcon arrow_rd_;

    QGridLayout* ds_grid_{nullptr};

    std::map<unsigned int, QPushButton*> line_buttons_;
    std::map<unsigned int, QPushButton*> direction_buttons_;

    QIcon& iconForDirection(LabelDirection direction);

    std::map<std::string, std::string> old_sources_;
};

}
