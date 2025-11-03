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

class JSONObjectParser;
class QLineEdit;
class QCheckBox;
class QGridLayout;
class DataTypeFormatSelectionWidget;

class JSONObjectParserWidget : public QWidget
{
    Q_OBJECT
  public slots:
    //void toggleActiveSlot ();

    void jsonContainerKeyChangedSlot();
    void jsonKeyChangedSlot();
    void jsonValueChangedSlot();

    void overrideDataSourceChangedSlot();
    void dataSourceVariableChangedSlot();

    void addNewMappingSlot();

    void mappingActiveChangedSlot();
    void mappingKeyChangedSlot();
    void mappingCommentChangedSlot();
    void mappingDBContentVariableChangedSlot();
    void mappingMandatoryChangedSlot();
    void mappingInArrayChangedSlot();
    void mappingAppendChangedSlot();
    void mappingDeleteSlot();

  public:
    explicit JSONObjectParserWidget(JSONObjectParser& parser, QWidget* parent = nullptr);

    void setParser(JSONObjectParser& parser);
    void updateActive();
    void updateMappingsGrid();

  private:
    JSONObjectParser* parser_{nullptr};

    QCheckBox* active_check_ {nullptr};
    QLineEdit* json_container_key_edit_{nullptr};  // location of container with target report data
    QLineEdit* json_key_edit_{nullptr};            // * for all
    QLineEdit* json_value_edit_{nullptr};

    QCheckBox* override_data_source_check_{nullptr};
    QLineEdit* data_source_variable_name_edit_{nullptr};

    QGridLayout* mappings_grid_{nullptr};

    void update();
};
