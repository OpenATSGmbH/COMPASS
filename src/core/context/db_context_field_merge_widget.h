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

#include <json.hpp>

#include <QWidget>

#include <string>
#include <vector>

class QGridLayout;
class QLabel;

namespace context
{

/**
 * Per-field merge widget for Modified items in the merge dialog.
 *
 * Shows a table:
 *   Field Name | Config Value | [→] | Editable Value | [←] | DB Value
 *
 * Each row represents a differing field (1 level deep into sub-objects).
 * Simple values are shown as text, complex values as indented JSON.
 * Arrow buttons copy the respective value into the editable field.
 * The editable field defaults to the value from whichever context has
 * the later "modified" timestamp.
 */
class FieldMergeWidget : public QWidget
{
    Q_OBJECT

signals:
    void configChosenSignal();   // user clicked → on a field
    void dbChosenSignal();       // user clicked ← on a field
    void valueEditedSignal();    // user manually edited a field

public:
    FieldMergeWidget(QWidget* parent = nullptr);

    void show(const std::string& item_name,
              const nlohmann::json& config_json,
              const nlohmann::json& db_json,
              bool prefer_db,
              const QString& config_label = "Configuration",
              const QString& db_label = "Database");

    void clear();

    /// Returns the merged JSON built from the edited fields.
    nlohmann::json mergedJSON() const;

private:
    struct FieldRow
    {
        std::string path;          // e.g. "name", "info.latitude"
        nlohmann::json config_val; // value in Configuration
        nlohmann::json db_val;     // value in Database
        QString config_text;       // display text for config
        QString db_text;           // display text for db
        QWidget* edit_widget{nullptr}; // QLineEdit or QPlainTextEdit
        bool is_multiline{false};
    };

    void updateResultColor(FieldRow& row);

    void addRow(int row, const std::string& path,
                const nlohmann::json& config_val,
                const nlohmann::json& db_val,
                bool prefer_db);

    static QString valueToDisplay(const nlohmann::json& val);
    static nlohmann::json displayToValue(const QString& text, const nlohmann::json& reference);
    static bool jsonEqual(const nlohmann::json& a, const nlohmann::json& b);

    QLabel* title_label_{nullptr};
    QLabel* config_header_label_{nullptr};
    QLabel* db_header_label_{nullptr};
    QGridLayout* grid_{nullptr};
    std::vector<FieldRow> rows_;
};

} // namespace context
