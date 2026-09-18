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

#include <vector>
#include <cstddef>

class ASTERIXImportSource;
struct ASTERIXDecodeStatus;

class QTreeWidget;
class QTreeWidgetItem;
class QProgressBar;
class QLabel;
class QPushButton;

/**
 * Progress dialog of a file based ASTERIX import.
 *
 * Lists the used files in the same style as the file list of the import dialog:
 * one row per file, columns Filename / Size (MB) / Status. The row data is read
 * from the import source by const reference, the progress values come from the
 * decoder via ASTERIXDecodeStatus.
 */
class ASTERIXImportProgressDialog : public QDialog
{
    Q_OBJECT
public:
    ASTERIXImportProgressDialog(const ASTERIXImportSource& source, QWidget* parent = nullptr);
    virtual ~ASTERIXImportProgressDialog();

    void updateStatus(const ASTERIXDecodeStatus& status);
    void setMessage(const QString& message);

    bool wasAborted() const { return aborted_; }

    static const int DialogMinWidth     = 900;
    static const int MinVisibleFileRows = 3;
    static const int MaxVisibleFileRows = 12;

protected:
    // escape and the window close button abort the import, as in the previous progress dialog
    virtual void reject() override;

private:
    void createUI();
    void createFileRows();
    void updateTreeHeight();

    void abortSlot();

    const ASTERIXImportSource& source_;

    QTreeWidget*  file_tree_       {nullptr};
    QProgressBar* progress_bar_    {nullptr};
    QLabel*       message_label_   {nullptr};
    QLabel*       elapsed_label_   {nullptr};
    QLabel*       remaining_label_ {nullptr};
    QLabel*       records_label_   {nullptr};
    QPushButton*  abort_button_    {nullptr};

    std::vector<QTreeWidgetItem*> file_items_;   // row per used file
    std::vector<std::size_t>      file_indices_; // index of the row file in the source file list

    int  current_row_ {-1}; // row of the file being decoded, to scroll it into view on change
    bool aborted_     {false};
};
