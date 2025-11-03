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
#include <QTreeView>
#include <QStackedWidget>

#include <memory>

#include <boost/optional.hpp>

#include "json_fwd.hpp"

class EvaluationManager;
class EvaluationManagerWidget;

class QPushButton;
class QMenu;

class EvaluationResultsTabWidget : public QWidget
{
    Q_OBJECT

public slots:
    void itemClickedSlot(const QModelIndex& index);

    void stepBackSlot();

public:
    EvaluationResultsTabWidget(EvaluationManager& eval_man, EvaluationManagerWidget& man_widget);
    virtual ~EvaluationResultsTabWidget();

    void expand();

    void showResultWidget(QWidget* widget); // can be nullptr

    void selectId (const std::string& id, bool show_figure = false);
    void reshowLastId ();

    void showFigure(const QModelIndex& index);

protected:
    EvaluationManager& eval_man_;
    EvaluationManagerWidget& man_widget_;

    QTreeView* tree_view_;

    QStackedWidget* results_widget_{nullptr};

    QPushButton* sections_button_{nullptr};
    std::unique_ptr<QMenu> sections_menu_;

    QPushButton* back_button_ {nullptr};
    std::vector<std::string> id_history_;

    void expandAllParents (QModelIndex index);
    void updateBackButton ();
};
