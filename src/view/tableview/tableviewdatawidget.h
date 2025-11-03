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

#include "viewdatawidget.h"

class TableView;
class TableViewWidget;
class TableViewDataSource;
class QTabWidget;
class AllBufferTableWidget;
class BufferTableWidget;
class Buffer;
class DBContent;

class TableViewDataWidget : public ViewDataWidget
{
    Q_OBJECT

signals:
    void exportDoneSignal(bool cancelled);
public slots:
    void exportDataSlot();
    void exportDoneSlot(bool cancelled);

public:
    TableViewDataWidget(TableViewWidget* view_widget, 
                          QWidget* parent = nullptr, 
                          Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~TableViewDataWidget();

    void resetModels();
    void updateToSelection();

    void selectFirstSelectedRow();

    AllBufferTableWidget* getAllBufferTableWidget ();

    void updateToSettingsChange();

    void showTab(QWidget* widget_ptr, bool value);

protected:
    virtual void toolChanged_impl(int mode) override;
    virtual void loadingStarted_impl() override;
    virtual void loadingDone_impl() override;
    virtual void updateData_impl(bool requires_reset) override;
    virtual void clearData_impl() override;
    virtual void clearIntermediateRedrawData_impl() override;
    virtual DrawState redrawData_impl(bool recompute) override;
    virtual void liveReload_impl() override;
    virtual bool hasAnnotations_impl() const override { return false; }

    void viewInfoJSON_impl(nlohmann::json& info) const override;

    TableView*           view_{nullptr};
    TableViewDataSource* data_source_{nullptr};
    QTabWidget*            tab_widget_{nullptr};

    AllBufferTableWidget*  all_buffer_table_widget_{nullptr};

    std::map<std::string, BufferTableWidget*> buffer_tables_;
};

