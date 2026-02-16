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

#include "traced_assert.h"

#include <jasterix/jasterix.h>
#include <jasterix/spfedition.h>

#include <QComboBox>
#include <memory>

#include "asteriximporttask.h"

class ASTERIXSPFEditionComboBox : public QComboBox
{
    Q_OBJECT

  public slots:
    void changedSPFEditionSlot(const QString& edition)
    {
        emit changedSPFSignal(category_->number(), edition.toStdString());
    }

  signals:
    /// @brief Emitted if REF was changed
    void changedSPFSignal(const std::string& cat_str, const std::string& ref_ed_str);

  public:
    /// @brief Constructor
    ASTERIXSPFEditionComboBox(ASTERIXImportTask& task,
                              const std::shared_ptr<jASTERIX::Category> category,
                              QWidget* parent = nullptr)
        : QComboBox(parent), task_(task), category_(category)
    {
        addItem("");

        if (category_->spfEditions().size())
        {
            for (auto& spf_it : category_->spfEditions())
            {
                addItem(spf_it.first.c_str());
            }

            setCurrentIndex(0);
            connect(this, SIGNAL(activated(const QString&)), this,
                    SLOT(changedSPFEditionSlot(const QString&)));
        }
        else
            setDisabled(true);
    }
    /// @brief Destructor
    virtual ~ASTERIXSPFEditionComboBox() {}

    /// @brief Returns the currently selected framing
    std::string getSPFEdition() { return currentText().toStdString(); }

    /// @brief Sets the currently selected edition
    void setSPFEdition(const std::string& spf_ed_str)
    {
        int index = findText(QString(spf_ed_str.c_str()));
        traced_assert(index >= 0);
        setCurrentIndex(index);
    }

  protected:
    ASTERIXImportTask& task_;
    const std::shared_ptr<jASTERIX::Category> category_;
};
