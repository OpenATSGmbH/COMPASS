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

#include <QDialog>

#include <memory>

class FFTManager;
class FFTTableModel;
class FFTEditWidget;

class QTableView;
class QSortFilterProxyModel;

class FFTsConfigurationDialog : public QDialog
{
    Q_OBJECT

signals:
    void doneSignal();

public slots:

    void currentRowChanged(const QModelIndex& current, const QModelIndex& previous);

    void newFFTClickedSlot();

    void importClickedSlot();
    void deleteAllClickedSlot();
    void exportClickedSlot();
    void doneClickedSlot();

public:
    FFTsConfigurationDialog(FFTManager& ds_man);

    void updateFFT(const std::string& name);
    void beginResetModel();
    void endResetModel();

protected:
    FFTManager& fft_man_;

    QTableView* table_view_{nullptr};
    QSortFilterProxyModel* proxy_model_{nullptr};
    FFTTableModel* table_model_;

    FFTEditWidget* edit_widget_ {nullptr};
};

