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

#include <jasterix/category.h>
#include <jasterix/jasterix.h>
#include "logger.h"
#include "traced_assert.h"

#include <QComboBox>
#include <memory>
#include <string>

#include "asteriximporttask.h"

class ASTERIXEditionComboBox : public QComboBox
{
    Q_OBJECT

  public slots:
    void changedEditionSlot(const QString& edition)
    {
        emit changedEdition(category_->number(), edition.toStdString());
    }

  signals:
    /// @brief Emitted if edition was changed
    void changedEdition(const std::string& cat_str, const std::string& ed_str);

  public:
    /// @brief Constructor
    ASTERIXEditionComboBox(ASTERIXImportTask& task,
                           const std::shared_ptr<jASTERIX::Category> category,
                           QWidget* parent = nullptr)
        : QComboBox(parent), task_(task), category_(category)
    {
        for (const auto& ed_it : category_->editions())
        {
            logdbg << "cat " << category->number() << " ed " << ed_it.first;

            addItem(ed_it.first.c_str());
        }

        setCurrentIndex(0);
        connect(this, SIGNAL(activated(const QString&)), this,
                SLOT(changedEditionSlot(const QString&)));
    }
    /// @brief Destructor
    virtual ~ASTERIXEditionComboBox() {}

    /// @brief Returns the currently selected framing
    std::string getEdition() { return currentText().toStdString(); }

    /// @brief Sets the currently selected edition
    void setEdition(const std::string& edition)
    {
        int index = findText(QString(edition.c_str()));
        traced_assert(index >= 0);
        setCurrentIndex(index);
    }

  protected:
    ASTERIXImportTask& task_;
    const std::shared_ptr<jASTERIX::Category> category_;
};
