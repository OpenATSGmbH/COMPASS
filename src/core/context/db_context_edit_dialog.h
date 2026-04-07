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

class QComboBox;
class QTreeView;
class QStackedWidget;
class QPushButton;
class DataSourceEditWidget;
class ASTERIXConfigWidget;

namespace context { class SectorEditWidget; class FFTEditWidget; }

namespace context
{

class DBContextManager;
class DBContextEditTreeModel;

/**
 * Main edit dialog for DBContexts. Shows a context selector at the top,
 * a tree view of context items on the left, and detail widgets on the right.
 */
class DBContextEditDialog : public QDialog
{
    Q_OBJECT

public:
    explicit DBContextEditDialog(DBContextManager& manager, QWidget* parent = nullptr);
    ~DBContextEditDialog() override;

private slots:
    void contextComboChangedSlot(const QString& name);
    void itemClickedSlot(const QModelIndex& index);
    void showContextMenuSlot(const QPoint& pos);
    void copySlot();
    void renameSlot();
    void deleteSlot();

private:
    void rebuildTree();
    void rebuildContextCombo();
    void showDetailWidget(QWidget* widget);
    QWidget* createPlaceholderLabel(const QString& text);

    // context menu handlers
    void showDataSourcesGroupMenu();
    void showDataSourceItemMenu(unsigned int ds_id);
    void showSectorLayersGroupMenu();
    void showSectorLayerMenu(const std::string& layer_name);
    void showSectorItemMenu(unsigned int sector_id);
    void showFFTsGroupMenu();
    void showFFTItemMenu(const std::string& fft_name);

    DBContextManager& manager_;

    QComboBox* context_combo_{nullptr};
    QPushButton* copy_button_{nullptr};
    QPushButton* rename_button_{nullptr};
    QPushButton* delete_button_{nullptr};

    QTreeView* tree_view_{nullptr};
    QStackedWidget* detail_stack_{nullptr};

    DBContextEditTreeModel* tree_model_{nullptr};

    // detail widgets (lazy, owned by stacked widget)
    DataSourceEditWidget* ds_edit_widget_{nullptr};
    ASTERIXConfigWidget* asterix_widget_{nullptr};
    SectorEditWidget* sector_edit_widget_{nullptr};
    FFTEditWidget* fft_edit_widget_{nullptr};
};

} // namespace context
