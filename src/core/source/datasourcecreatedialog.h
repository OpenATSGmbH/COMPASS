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

class DataSourceManager;
class DataSourcesConfigurationDialog;
class DSTypeSelectionComboBox;

class QLineEdit;
class QPushButton;

class DataSourceCreateDialog : public QDialog
{
    Q_OBJECT

signals:
    void doneSignal();

public slots:
    void dsTypeEditedSlot(const QString& value);

    void sacEditedSlot(const QString& value_str);
    void sicEditedSlot(const QString& value_str);

    void cancelClickedSlot();
    void doneClickedSlot();

public:
    DataSourceCreateDialog(DataSourcesConfigurationDialog& dialog, DataSourceManager& ds_man);
    virtual ~DataSourceCreateDialog();

    unsigned int sac() const;

    unsigned int sic() const;

    std::string dsType() const;

    bool cancelled() const;

protected:
    DataSourceManager& ds_man_;

    DSTypeSelectionComboBox* dstype_combo_{nullptr};

    QLineEdit* sac_edit_{nullptr};
    QLineEdit* sic_edit_{nullptr};

    QPushButton* cancel_button_{nullptr};
    QPushButton* done_button_{nullptr};

    std::string ds_type_;
    unsigned int sac_ {0};
    unsigned int sic_ {0};

    bool cancelled_{false};

    void checkInput();
};
