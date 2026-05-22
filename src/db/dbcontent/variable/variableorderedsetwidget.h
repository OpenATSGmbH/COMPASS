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

class QListWidget;

#include <QMenu>
#include <QWidget>

#include "dbcontent/variable/variableorderedset.h"

#include "test/ui_test_testable.h"

namespace dbContent
{

class VariableOrderedSetWidget : public QWidget, public ui_test::UITestable
{
    Q_OBJECT

public slots:
    void updateVariableListSlot();
    void removeSlot();
    void moveUpSlot();
    void moveDownSlot();

protected slots:
    void triggerSlot(QAction* action);
    void showMenuSlot();

public:
    VariableOrderedSetWidget(VariableOrderedSet& set, QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~VariableOrderedSetWidget();

    const QListWidget* listWidget() const { return list_widget_; }
    const VariableOrderedSet& variableSet() const { return set_; }

    void setVariables(const std::vector<std::pair<std::string,std::string>>& vars);

    boost::optional<QString> uiGet(const QString& what = QString()) const override;
    nlohmann::json uiGetJSON(const QString& what = QString()) const override;
    bool uiSet(const QString& str) override;

    static const std::string VariableSeparator;

protected:
    VariableOrderedSet& set_;

    QListWidget* list_widget_{nullptr};
    int current_index_{-1};
};

}
