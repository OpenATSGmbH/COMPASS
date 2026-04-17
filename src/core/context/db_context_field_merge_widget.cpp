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
#include <QTextBlock>
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

static const size_t MaxDumpLength = 200;
static const int MaxDepth = 5;

/// Recursively collect field diffs, descending into sub-objects when the
/// dump size exceeds MaxDumpLength. One-sided values (only in config or db)
/// are never split — shown as a single row regardless of size.
void collectDiffs(const json& config_j, const json& db_j,
                  const std::string& prefix,
                  std::vector<std::tuple<std::string, json, json>>& out,
                  int depth = 0)
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
        std::string path = prefix.empty() ? key : prefix + "." + key;

        json cv = config_j.contains(key) ? config_j.at(key) : json();
        json dv = db_j.contains(key) ? db_j.at(key) : json();

        if (jsonEqualNormalized(cv, dv))
            continue;

        // one-sided: never recurse, show as single row
        if (cv.is_null() || dv.is_null())
        {
            out.emplace_back(path, cv, dv);
            continue;
        }

        // both sides exist and differ — check if we should recurse
        bool both_objects = cv.is_object() && dv.is_object();
        bool too_large = cv.dump().size() > MaxDumpLength || dv.dump().size() > MaxDumpLength;

        if (both_objects && too_large && depth < MaxDepth)
        {
            collectDiffs(cv, dv, path, out, depth + 1);
        }
        else
        {
            out.emplace_back(path, cv, dv);
        }
    }
}

} // anonymous namespace

FieldMergeWidget::FieldMergeWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);

    title_label_ = new QLabel();
    QFont title_font = title_label_->font();
    title_font.setBold(true);
    title_label_->setFont(title_font);
    outer->addWidget(title_label_);

    auto* scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    auto* inner = new QWidget();
    auto* inner_layout = new QVBoxLayout(inner);
    inner_layout->setContentsMargins(0, 0, 0, 0);

    grid_ = new QGridLayout();
    grid_->setColumnStretch(0, 1); // config value
    grid_->setColumnStretch(1, 0); // arrow button
    grid_->setColumnStretch(2, 1); // editable
    grid_->setColumnStretch(3, 0); // arrow button
    grid_->setColumnStretch(4, 1); // db value

    // header row
    auto makeBoldLabel = [](const QString& text)
    {
        auto* label = new QLabel(text);
        QFont f = label->font();
        f.setBold(true);
        label->setFont(f);
        return label;
    };

    config_header_label_ = makeBoldLabel("Configuration");
    db_header_label_     = makeBoldLabel("Database");

    grid_->addWidget(config_header_label_, 0, 0);
    grid_->addWidget(new QLabel(), 0, 1); // arrow column
    grid_->addWidget(makeBoldLabel("Result"), 0, 2);
    grid_->addWidget(new QLabel(), 0, 3); // arrow column
    grid_->addWidget(db_header_label_, 0, 4);

    inner_layout->addLayout(grid_, 1);

    scroll->setWidget(inner);
    outer->addWidget(scroll);
}

void FieldMergeWidget::show(const std::string& item_name,
                            const json& config_json, const json& db_json, bool prefer_db,
                            const QString& config_label, const QString& db_label)
{
    loginf << "showing item '" << item_name << "' prefer_db " << prefer_db;

    clear();

    title_label_->setText(QString::fromStdString(item_name));
    if (config_header_label_)
        config_header_label_->setText(config_label);
    if (db_header_label_)
        db_header_label_->setText(db_label);

    std::vector<std::tuple<std::string, json, json>> diffs;
    collectDiffs(config_json, db_json, "", diffs);

    int row = 1; // row 0 is header
    for (const auto& [path, cv, dv] : diffs)
    {
        addRow(row, path, cv, dv, prefer_db);
        row += 2; // each field takes 2 rows: name + values
    }
}

void FieldMergeWidget::clear()
{
    loginf << "clearing " << rows_.size() << " rows";

    rows_.clear();

    // remove all widgets from row 1 onwards (row 0 is header)
    int total_rows = grid_->rowCount();
    int total_cols = grid_->columnCount();

    logdbg << "grid has " << total_rows << " rows, " << total_cols << " cols";

    for (int row = 1; row < total_rows; ++row)
    {
        for (int col = 0; col < total_cols; ++col)
        {
            auto* item = grid_->itemAtPosition(row, col);
            if (item && item->widget())
            {
                QWidget* w = item->widget();
                grid_->removeWidget(w);
                delete w;
            }
        }
    }

    loginf << "clear done";
}

void FieldMergeWidget::addRow(int row, const std::string& path,
                               const json& config_val, const json& db_val,
                               bool prefer_db)
{
    FieldRow fr;
    fr.path = path;
    fr.config_val = config_val;
    fr.db_val = db_val;

    QFont mono_font("monospace");
    mono_font.setStyleHint(QFont::Monospace);

    bool is_complex = (config_val.is_object() || config_val.is_array() ||
                       db_val.is_object() || db_val.is_array());
    fr.is_multiline = is_complex;

    // field name — spans all columns, italic
    auto* name_label = new QLabel(QString::fromStdString(path));
    QFont italic_font = name_label->font();
    italic_font.setItalic(true);
    name_label->setFont(italic_font);
    name_label->setToolTip(QString::fromStdString(path));
    grid_->addWidget(name_label, row, 0, 1, 5); // span all 5 columns

    int val_row = row + 1;

    // full text for editable column
    QString config_text = valueToDisplay(config_val);
    QString db_text = valueToDisplay(db_val);
    const json& default_val = prefer_db ? db_val : config_val;
    QString default_text = valueToDisplay(default_val);

    // for read-only columns: truncate one-sided large values
    auto truncate = [](const QString& text, const json& this_val, const json& other_val) -> QString
    {
        // only truncate if other side is null (one-sided) and value is large
        if (!other_val.is_null())
            return text;
        if (text.size() <= static_cast<int>(MaxDumpLength))
            return text;

        return text.left(static_cast<int>(MaxDumpLength)) + "\n...";
    };

    QString config_display = truncate(config_text, config_val, db_val);
    QString db_display = truncate(db_text, db_val, config_val);

    // helper: create a read-only QPlainTextEdit
    auto makeReadOnly = [&mono_font](const QString& text)
    {
        auto* te = new QPlainTextEdit();
        te->setPlainText(text);
        te->setReadOnly(true);
        te->setFont(mono_font);
        te->setFrameShape(QFrame::NoFrame);

        // size to content
        QFontMetrics fm(mono_font);
        int line_count = std::max(1, text.count('\n') + 1);
        int height = fm.lineSpacing() * line_count + 10;
        te->setMinimumHeight(height);

        return te;
    };

    // helper: highlight differing lines between two text edits
    auto highlightDiffs = [](QPlainTextEdit* config_te, QPlainTextEdit* db_te)
    {
        QStringList config_lines = config_te->toPlainText().split('\n');
        QStringList db_lines = db_te->toPlainText().split('\n');

        int max_lines = std::max(config_lines.size(), db_lines.size());

        QColor config_color(200, 255, 200); // light green
        QColor db_color(200, 220, 255);     // light blue

        auto selectBlock = [](QPlainTextEdit* te, int line, QColor color) -> QTextEdit::ExtraSelection
        {
            QTextBlock block = te->document()->findBlockByNumber(line);
            QTextCursor cursor(block);
            cursor.movePosition(QTextCursor::StartOfBlock);
            cursor.movePosition(QTextCursor::EndOfBlock, QTextCursor::KeepAnchor);

            QTextEdit::ExtraSelection sel;
            sel.format.setBackground(color);
            sel.format.setProperty(QTextFormat::FullWidthSelection, true);
            sel.cursor = cursor;
            return sel;
        };

        QList<QTextEdit::ExtraSelection> config_sels, db_sels;

        for (int i = 0; i < max_lines; ++i)
        {
            QString cl = i < config_lines.size() ? config_lines[i] : QString();
            QString dl = i < db_lines.size() ? db_lines[i] : QString();

            if (cl == dl)
                continue;

            if (i < config_lines.size())
                config_sels.append(selectBlock(config_te, i, config_color));

            if (i < db_lines.size())
                db_sels.append(selectBlock(db_te, i, db_color));
        }

        config_te->setExtraSelections(config_sels);
        db_te->setExtraSelections(db_sels);
    };

    // config value (read-only)
    auto* config_edit = makeReadOnly(config_display);
    grid_->addWidget(config_edit, val_row, 0);

    // editable result
    QWidget* edit_widget = nullptr;
    if (is_complex)
    {
        auto* te = new QPlainTextEdit();
        te->setPlainText(default_text);
        te->setFont(mono_font);

        QFontMetrics fm(mono_font);
        int line_count = std::max(1, default_text.count('\n') + 1);
        int height = fm.lineSpacing() * line_count + 10;
        te->setMinimumHeight(height);

        edit_widget = te;
    }
    else
    {
        auto* le = new QLineEdit();
        le->setText(default_text);
        edit_widget = le;
    }
    fr.edit_widget = edit_widget;

    // connect edit signals — update color + emit signal
    size_t row_idx = rows_.size(); // index this row will have after push_back
    if (is_complex)
    {
        connect(static_cast<QPlainTextEdit*>(edit_widget), &QPlainTextEdit::textChanged,
                this, [this, row_idx]()
        {
            if (row_idx < rows_.size())
                updateResultColor(rows_[row_idx]);
            emit valueEditedSignal();
        });
    }
    else
    {
        connect(static_cast<QLineEdit*>(edit_widget), &QLineEdit::textEdited,
                this, [this, row_idx]()
        {
            if (row_idx < rows_.size())
                updateResultColor(rows_[row_idx]);
            emit valueEditedSignal();
        });
    }

    // [→] button: copy config value to editable
    auto* to_right = new QPushButton(QString::fromUtf8("\u2192"));
    to_right->setFixedWidth(30);
    to_right->setToolTip("Use Configuration value");
    connect(to_right, &QPushButton::clicked, this, [this, edit_widget, config_text, is_complex, row_idx]()
    {
        edit_widget->blockSignals(true);
        if (is_complex)
            static_cast<QPlainTextEdit*>(edit_widget)->setPlainText(config_text);
        else
            static_cast<QLineEdit*>(edit_widget)->setText(config_text);
        edit_widget->blockSignals(false);
        if (row_idx < rows_.size())
            updateResultColor(rows_[row_idx]);
        emit configChosenSignal();
    });

    // [←] button: copy db value to editable
    auto* to_left = new QPushButton(QString::fromUtf8("\u2190"));
    to_left->setFixedWidth(30);
    to_left->setToolTip("Use Database value");
    connect(to_left, &QPushButton::clicked, this, [this, edit_widget, db_text, is_complex, row_idx]()
    {
        edit_widget->blockSignals(true);
        if (is_complex)
            static_cast<QPlainTextEdit*>(edit_widget)->setPlainText(db_text);
        else
            static_cast<QLineEdit*>(edit_widget)->setText(db_text);
        edit_widget->blockSignals(false);
        if (row_idx < rows_.size())
            updateResultColor(rows_[row_idx]);
        emit dbChosenSignal();
    });

    grid_->addWidget(to_right, val_row, 1);
    grid_->addWidget(edit_widget, val_row, 2);
    grid_->addWidget(to_left, val_row, 3);

    // db value (read-only)
    auto* db_edit = makeReadOnly(db_display);
    grid_->addWidget(db_edit, val_row, 4);

    // highlight differing lines
    highlightDiffs(config_edit, db_edit);

    // store display texts for color comparison
    fr.config_text = config_text;
    fr.db_text = db_text;

    rows_.push_back(fr);

    // set initial color
    updateResultColor(rows_.back());
}

void FieldMergeWidget::updateResultColor(FieldRow& row)
{
    if (!row.is_multiline)
    {
        // single-line: color the whole widget
        QString current_text = static_cast<QLineEdit*>(row.edit_widget)->text();

        QString bg;
        if (current_text == row.config_text)
            bg = "background-color: rgb(200, 255, 200);"; // green — matches config
        else if (current_text == row.db_text)
            bg = "background-color: rgb(200, 220, 255);"; // blue — matches db
        else
            bg = "background-color: rgb(255, 210, 210);"; // red — custom

        row.edit_widget->setStyleSheet(bg);
        return;
    }

    // multiline: highlight only lines where config and db differ,
    // colored by which side the result line matches
    auto* te = static_cast<QPlainTextEdit*>(row.edit_widget);
    te->setStyleSheet(""); // clear any whole-widget color

    QStringList result_lines = te->toPlainText().split('\n');
    QStringList config_lines = row.config_text.split('\n');
    QStringList db_lines = row.db_text.split('\n');

    QColor config_color(200, 255, 200); // green — matches config
    QColor db_color(200, 220, 255);     // blue — matches db
    QColor custom_color(255, 210, 210); // red — matches neither

    QList<QTextEdit::ExtraSelection> sels;

    for (int i = 0; i < result_lines.size(); ++i)
    {
        QString cl = i < config_lines.size() ? config_lines[i] : QString();
        QString dl = i < db_lines.size() ? db_lines[i] : QString();

        // skip lines that are identical on both sides — not a conflict
        if (cl == dl)
            continue;

        QTextBlock block = te->document()->findBlockByNumber(i);
        if (!block.isValid())
            continue;

        const QString& rl = result_lines[i];

        QTextCursor cursor(block);
        cursor.movePosition(QTextCursor::StartOfBlock);
        cursor.movePosition(QTextCursor::EndOfBlock, QTextCursor::KeepAnchor);

        QTextEdit::ExtraSelection sel;
        sel.format.setProperty(QTextFormat::FullWidthSelection, true);
        sel.cursor = cursor;

        if (rl == cl)
            sel.format.setBackground(config_color);
        else if (rl == dl)
            sel.format.setBackground(db_color);
        else
            sel.format.setBackground(custom_color);

        sels.append(sel);
    }

    te->setExtraSelections(sels);
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
    {
        // fixed-point only (never scientific). Use generous precision then
        // strip trailing zeros; keep at least one decimal so it stays float-ish.
        QString s = QString::number(val.get<double>(), 'f', 12);
        if (s.contains('.'))
        {
            int end = s.size();
            while (end > 1 && s[end - 1] == QChar('0'))
                --end;
            if (end > 1 && s[end - 1] == QChar('.'))
                ++end; // keep one trailing zero after the dot
            s.truncate(end);
        }
        return s;
    }
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
