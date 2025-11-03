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

#include <jasterix/jasterix.h>

#include <QWidget>
#include <memory>

class ASTERIXImportTask;

class QVBoxLayout;
class QGridLayout;
class QComboBox;
class QPushButton;
class ASTERIXFramingComboBox;

class ASTERIXConfigWidget : public QWidget
{
    Q_OBJECT

  signals:

  public slots:
    void framingChangedSlot();
    void framingEditSlot();

    void categoryCheckedSlot();
    void editionChangedSlot(const std::string& cat_str, const std::string& ed_str);
    void refEditionChangedSlot(const std::string& cat_str, const std::string& ed_str);
    void spfEditionChangedSlot(const std::string& cat_str, const std::string& ed_str);
    void categoryEditionEditSlot();
    void categoryREFEditionEditSlot();
    void categorySPFEditionEditSlot();
    //void categoryMappingChangedSlot(unsigned int cat, const std::string& mapping_str);

    void editDataBlockSlot();
    void editCategoriesSlot();
    void refreshjASTERIXSlot();

    void updateSlot();

  public:
    ASTERIXConfigWidget(ASTERIXImportTask& task, QWidget* parent = nullptr);
    virtual ~ASTERIXConfigWidget();

  protected:
    ASTERIXImportTask& task_;

    QVBoxLayout* main_layout_{nullptr};
    QGridLayout* categories_grid_{nullptr};

    ASTERIXFramingComboBox* framing_combo_{nullptr};
    QPushButton* framing_edit_{nullptr};

    std::map<unsigned int, QPushButton*> ref_edit_buttons_;
    std::map<unsigned int, QPushButton*> spf_edit_buttons_;

    void updateFraming();
    void updateCategories();
};
