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

#include "db_context_field_merge_widget.h"
#include "logger.h"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

using namespace nlohmann;

namespace context
{

namespace
{

/// Normalize a JSON value for comparison: re-parse to remove formatting differences.
json normalize(const json& val)
{
    if (val.is_string())
    {
        // try to parse string as JSON in case it was stored with different formatting
        try
        {
            return json::parse(val.get<std::string>());
        }
        catch (...)
        {
            return val;
        }
    }
    return val;
}

/// Compare two JSON values, ignoring whitespace/formatting differences.
bool jsonEqualNormalized(const json& a, const json& b)
{
    return normalize(a) == normalize(b);
}

/// Collect field diffs with 1-level descent into sub-objects.
/// Top-level primitive keys get their own row.
/// Top-level object keys have their children expanded one level.
/// Deeper nesting is shown as JSON text.
void collectShallowDiffs(const json& config_j, const json& db_j,
                         std::vector<std::tuple<std::string, json, json>>& out)
{
    // gather all keys from both sides
    std::set<std::string> all_keys;
    if (config_j.is_object())
        for (auto it = config_j.begin(); it != config_j.end(); ++it)
            all_keys.insert(it.key());
    if (db_j.is_object())
        for (auto it = db_j.begin(); it != db_j.end(); ++it)
            all_keys.insert(it.key());

    for (const auto& key : all_keys)
    {
        json cv = config_j.contains(key) ? config_j.at(key) : json();
        json dv = db_j.contains(key) ? db_j.at(key) : json();

        // if both are objects, descend one level
        if (cv.is_object() && dv.is_object())
        {
            std::set<std::string> sub_keys;
            for (auto it = cv.begin(); it != cv.end(); ++it)
                sub_keys.insert(it.key());
            for (auto it = dv.begin(); it != dv.end(); ++it)
                sub_keys.insert(it.key());

            for (const auto& sk : sub_keys)
            {
                json scv = cv.contains(sk) ? cv.at(sk) : json();
                json sdv = dv.contains(sk) ? dv.at(sk) : json();

                if (!jsonEqualNormalized(scv, sdv))
                    out.emplace_back(key + "." + sk, scv, sdv);
            }
        }
        else if (!jsonEqualNormalized(cv, dv))
        {
            out.emplace_back(key, cv, dv);
        }
    }
}

} // anonymous namespace

FieldMergeWidget::FieldMergeWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);

    auto* scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    auto* inner = new QWidget();
    auto* inner_layout = new QVBoxLayout(inner);
    inner_layout->setContentsMargins(0, 0, 0, 0);

    grid_ = new QGridLayout();
    grid_->setColumnStretch(0, 1); // field name
    grid_->setColumnStretch(1, 2); // config value
    grid_->setColumnStretch(2, 0); // arrow button
    grid_->setColumnStretch(3, 2); // editable
    grid_->setColumnStretch(4, 0); // arrow button
    grid_->setColumnStretch(5, 2); // db value

    // header row
    auto makeBoldLabel = [](const QString& text)
    {
        auto* label = new QLabel(text);
        QFont f = label->font();
        f.setBold(true);
        label->setFont(f);
        return label;
    };

    grid_->addWidget(makeBoldLabel("Field"), 0, 0);
    grid_->addWidget(makeBoldLabel("Configuration"), 0, 1);
    grid_->addWidget(new QLabel(), 0, 2); // arrow column
    grid_->addWidget(makeBoldLabel("Result"), 0, 3);
    grid_->addWidget(new QLabel(), 0, 4); // arrow column
    grid_->addWidget(makeBoldLabel("Database"), 0, 5);

    inner_layout->addLayout(grid_);
    inner_layout->addStretch();

    scroll->setWidget(inner);
    outer->addWidget(scroll);
}

void FieldMergeWidget::show(const json& config_json, const json& db_json, bool prefer_db)
{
    clear();

    std::vector<std::tuple<std::string, json, json>> diffs;
    collectShallowDiffs(config_json, db_json, diffs);

    int row = 1; // row 0 is header
    for (const auto& [path, cv, dv] : diffs)
        addRow(row++, path, cv, dv, prefer_db);
}

void FieldMergeWidget::clear()
{
    // remove all rows except the header (row 0)
    for (auto& r : rows_)
    {
        // widgets are owned by the grid layout, no manual delete needed
    }
    rows_.clear();

    // remove all items from row 1 onwards
    while (grid_->rowCount() > 1)
    {
        int row = grid_->rowCount() - 1;
        for (int col = 0; col < grid_->columnCount(); ++col)
        {
            auto* item = grid_->itemAtPosition(row, col);
            if (item && item->widget())
            {
                item->widget()->setParent(nullptr);
                delete item->widget();
            }
        }
    }
}

void FieldMergeWidget::addRow(int row, const std::string& path,
                               const json& config_val, const json& db_val,
                               bool prefer_db)
{
    FieldRow fr;
    fr.path = path;
    fr.config_val = config_val;
    fr.db_val = db_val;

    // field name
    auto* name_label = new QLabel(QString::fromStdString(path));
    name_label->setToolTip(QString::fromStdString(path));
    grid_->addWidget(name_label, row, 0);

    // config value (read-only)
    QString config_text = valueToDisplay(config_val);
    auto* config_label = new QLabel(config_text);
    config_label->setWordWrap(true);
    config_label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    grid_->addWidget(config_label, row, 1);

    // determine if we need a multiline editor
    bool is_complex = (config_val.is_object() || config_val.is_array() ||
                       db_val.is_object() || db_val.is_array());
    fr.is_multiline = is_complex;

    // editable value — default to preferred side
    const json& default_val = prefer_db ? db_val : config_val;
    QString default_text = valueToDisplay(default_val);

    QWidget* edit_widget = nullptr;
    if (is_complex)
    {
        auto* te = new QPlainTextEdit();
        te->setPlainText(default_text);
        te->setMaximumHeight(120);
        edit_widget = te;
    }
    else
    {
        auto* le = new QLineEdit();
        le->setText(default_text);
        edit_widget = le;
    }
    fr.edit_widget = edit_widget;

    // connect edit signals to emit valueEditedSignal
    if (is_complex)
    {
        connect(static_cast<QPlainTextEdit*>(edit_widget), &QPlainTextEdit::textChanged,
                this, &FieldMergeWidget::valueEditedSignal);
    }
    else
    {
        connect(static_cast<QLineEdit*>(edit_widget), &QLineEdit::textEdited,
                this, &FieldMergeWidget::valueEditedSignal);
    }

    // [→] button: copy config value to editable
    auto* to_right = new QPushButton(QString::fromUtf8("\u2192"));
    to_right->setFixedWidth(30);
    to_right->setToolTip("Use Configuration value");
    connect(to_right, &QPushButton::clicked, this, [this, edit_widget, config_text, is_complex]()
    {
        if (is_complex)
        {
            edit_widget->blockSignals(true);
            static_cast<QPlainTextEdit*>(edit_widget)->setPlainText(config_text);
            edit_widget->blockSignals(false);
        }
        else
        {
            edit_widget->blockSignals(true);
            static_cast<QLineEdit*>(edit_widget)->setText(config_text);
            edit_widget->blockSignals(false);
        }
        emit configChosenSignal();
    });

    // [←] button: copy db value to editable
    QString db_text = valueToDisplay(db_val);
    auto* to_left = new QPushButton(QString::fromUtf8("\u2190"));
    to_left->setFixedWidth(30);
    to_left->setToolTip("Use Database value");
    connect(to_left, &QPushButton::clicked, this, [this, edit_widget, db_text, is_complex]()
    {
        if (is_complex)
        {
            edit_widget->blockSignals(true);
            static_cast<QPlainTextEdit*>(edit_widget)->setPlainText(db_text);
            edit_widget->blockSignals(false);
        }
        else
        {
            edit_widget->blockSignals(true);
            static_cast<QLineEdit*>(edit_widget)->setText(db_text);
            edit_widget->blockSignals(false);
        }
        emit dbChosenSignal();
    });

    grid_->addWidget(to_right, row, 2);
    grid_->addWidget(edit_widget, row, 3);
    grid_->addWidget(to_left, row, 4);

    // db value (read-only)
    auto* db_label = new QLabel(db_text);
    db_label->setWordWrap(true);
    db_label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    grid_->addWidget(db_label, row, 5);

    rows_.push_back(fr);
}

QString FieldMergeWidget::valueToDisplay(const json& val)
{
    if (val.is_null())
        return "(not set)";
    if (val.is_string())
        return QString::fromStdString(val.get<std::string>());
    if (val.is_number_integer())
        return QString::number(val.get<int64_t>());
    if (val.is_number_unsigned())
        return QString::number(val.get<uint64_t>());
    if (val.is_number_float())
        return QString::number(val.get<double>(), 'g', 12);
    if (val.is_boolean())
        return val.get<bool>() ? "true" : "false";

    // object or array: pretty-print
    return QString::fromStdString(val.dump(2));
}

json FieldMergeWidget::displayToValue(const QString& text, const json& reference)
{
    std::string str = text.trimmed().toStdString();

    if (str.empty() || str == "(not set)")
        return json();

    // if the reference is a complex type, try to parse as JSON
    if (reference.is_object() || reference.is_array())
    {
        try
        {
            return json::parse(str);
        }
        catch (...)
        {
            // fall through to string
        }
    }

    // try to match the reference type
    if (reference.is_boolean())
        return (str == "true" || str == "1");

    if (reference.is_number_integer() || reference.is_number_unsigned())
    {
        try { return json(std::stol(str)); }
        catch (...) {}
    }

    if (reference.is_number_float())
    {
        try { return json(std::stod(str)); }
        catch (...) {}
    }

    // default: string
    return json(str);
}

bool FieldMergeWidget::jsonEqual(const json& a, const json& b)
{
    return normalize(a) == normalize(b);
}

json FieldMergeWidget::mergedJSON() const
{
    // We don't have the full base JSON here — the caller should start from
    // one side and apply field overrides. Return a map of path -> edited value.
    json result = json::object();

    for (const auto& row : rows_)
    {
        QString text;
        if (row.is_multiline)
            text = static_cast<QPlainTextEdit*>(row.edit_widget)->toPlainText();
        else
            text = static_cast<QLineEdit*>(row.edit_widget)->text();

        // use whichever reference value is non-null for type inference
        const json& ref = !row.config_val.is_null() ? row.config_val : row.db_val;
        result[row.path] = displayToValue(text, ref);
    }

    return result;
}

} // namespace context
