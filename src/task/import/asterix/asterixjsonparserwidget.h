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

class ASTERIXJSONParser;
class ASTERIXJSONParserDetailWidget;

class QSplitter;
class QTableView;
class QSortFilterProxyModel;


class ASTERIXJSONParserWidget : public QWidget
{
    Q_OBJECT
public slots:
    void currentRowChanged(const QModelIndex& current, const QModelIndex& previous);

public:
    explicit ASTERIXJSONParserWidget(ASTERIXJSONParser& parser, QWidget* parent = nullptr);
    virtual ~ASTERIXJSONParserWidget();

    void resizeColumnsToContents();

    void selectModelRow (unsigned int row);

private:
    ASTERIXJSONParser& parser_;

    QSplitter* splitter_{nullptr};
    QTableView* table_view_{nullptr};
    QSortFilterProxyModel* proxy_model_{nullptr};

    ASTERIXJSONParserDetailWidget* detail_widget_{nullptr};

    virtual void keyPressEvent(QKeyEvent* event);
};
