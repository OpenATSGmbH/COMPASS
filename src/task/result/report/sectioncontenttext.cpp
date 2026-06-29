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

#include "task/result/report/sectioncontenttext.h"
#include "task/result/report/reportexporter.h"
#include "task/result/taskresult.h"

#include "taskmanager.h"
#include "logger.h"

#include <QResizeEvent>
#include <QString>
#include <QStringList>
#include <QTextDocument>
#include <QTextEdit>
#include <QVBoxLayout>

#include "traced_assert.h"

namespace ResultReport
{

const std::string SectionContentText::FieldTexts  = "texts";
const std::string SectionContentText::FieldBlocks = "blocks";

namespace
{
// Block-type <-> JSON string mapping (stable, human-readable in the export).
std::string blockTypeToString(SectionContentText::BlockType t)
{
    switch (t)
    {
        case SectionContentText::BlockType::Paragraph:   return "paragraph";
        case SectionContentText::BlockType::BulletList:  return "bullet_list";
        case SectionContentText::BlockType::OrderedList: return "ordered_list";
        case SectionContentText::BlockType::Note:        return "note";
    }
    return "paragraph";
}

SectionContentText::BlockType blockTypeFromString(const std::string& s)
{
    if (s == "bullet_list")  return SectionContentText::BlockType::BulletList;
    if (s == "ordered_list") return SectionContentText::BlockType::OrderedList;
    if (s == "note")         return SectionContentText::BlockType::Note;
    return SectionContentText::BlockType::Paragraph;
}

std::string noteLevelToString(SectionContentText::NoteLevel l)
{
    return l == SectionContentText::NoteLevel::Warning ? "warning" : "info";
}

SectionContentText::NoteLevel noteLevelFromString(const std::string& s)
{
    return s == "warning" ? SectionContentText::NoteLevel::Warning
                          : SectionContentText::NoteLevel::Info;
}

// Convert the legacy flat "texts" array into structured blocks, honoring the
// historic '- ' bullet convention so already-persisted reports keep rendering
// as lists. Used only when reading legacy JSON that has no "blocks" field.
std::vector<SectionContentText::Block> blocksFromLegacyTexts(
    const std::vector<std::string>& texts)
{
    using Block     = SectionContentText::Block;
    using BlockType = SectionContentText::BlockType;

    std::vector<Block> blocks;
    Block* list = nullptr; // open bullet-list block, if any

    for (const auto& text : texts)
    {
        const QStringList lines = QString::fromStdString(text).split('\n');
        for (const QString& raw : lines)
        {
            const QString line = raw.trimmed();
            if (line.isEmpty())
            {
                list = nullptr;
                continue;
            }
            if (line.startsWith("- "))
            {
                if (!list)
                {
                    blocks.push_back(Block{BlockType::BulletList, {}, {}});
                    list = &blocks.back();
                }
                list->items.push_back(line.mid(2).toStdString());
            }
            else
            {
                list = nullptr;
                blocks.push_back(Block{BlockType::Paragraph, {}, {line.toStdString()}});
            }
        }
    }
    return blocks;
}
}

/**
 */
SectionContentText::SectionContentText(unsigned int id,
                                       const std::string& name,
                                       Section* parent_section)
:   SectionContent(ContentType::Text, id, name, parent_section)
{
}

/**
 */
SectionContentText::SectionContentText(Section* parent_section)
:   SectionContent(ContentType::Text, parent_section)
{
}

/**
 */
void SectionContentText::addText(const std::string& text)
{
    // Split on '\n' into one paragraph per non-empty line.
    const QStringList lines = QString::fromStdString(text).split('\n');
    for (const QString& raw : lines)
    {
        const QString line = raw.trimmed();
        if (line.isEmpty())
            continue;
        blocks_.push_back(Block{BlockType::Paragraph, {}, {line.toStdString()}});
    }
}

/**
 */
void SectionContentText::addList(const std::vector<std::string>& items)
{
    if (items.empty())
        return;
    blocks_.push_back(Block{BlockType::BulletList, {}, items});
}

/**
 */
void SectionContentText::addOrderedList(const std::vector<std::string>& items)
{
    if (items.empty())
        return;
    blocks_.push_back(Block{BlockType::OrderedList, {}, items});
}

/**
 */
void SectionContentText::addNote(const std::string& text, NoteLevel level)
{
    blocks_.push_back(Block{BlockType::Note, level, {text}});
}

/**
 */
std::string SectionContentText::resourceExtension() const
{
    return ReportExporter::ExportTextFormat;
}

namespace
{
// Read-only text view that sizes to its document height. Used as the
// section's prose container so the report flows naturally with no scroll
// bars and no fixed-height guesswork.
class FittingTextView : public QTextEdit
{
public:
    explicit FittingTextView(QWidget* parent = nullptr) : QTextEdit(parent)
    {
        setReadOnly(true);
        setFrameShape(QFrame::NoFrame);
        setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        document()->setDocumentMargin(0);
        viewport()->setAutoFillBackground(false);
        setStyleSheet("QTextEdit { background: transparent; }");
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    }

    QSize sizeHint() const override
    {
        const int w = viewport()->width() > 0
                          ? viewport()->width()
                          : QTextEdit::sizeHint().width();
        document()->setTextWidth(w);
        return { w, static_cast<int>(document()->size().height()) + 2 };
    }

    QSize minimumSizeHint() const override { return sizeHint(); }

protected:
    void resizeEvent(QResizeEvent* e) override
    {
        QTextEdit::resizeEvent(e);
        document()->setTextWidth(viewport()->width());
        updateGeometry();
    }
};

// Build an HTML rendering of the structured blocks: paragraphs become <p>,
// bullet lists <ul>, numbered lists <ol>, notes a tinted box. All content is
// HTML-escaped.
QString blocksToHtml(const std::vector<SectionContentText::Block>& blocks)
{
    using BlockType = SectionContentText::BlockType;
    using NoteLevel = SectionContentText::NoteLevel;

    auto esc = [](const std::string& s) {
        return QString::fromStdString(s).toHtmlEscaped();
    };

    QString html;

    for (const auto& block : blocks)
    {
        switch (block.type)
        {
            case BlockType::Paragraph:
                if (!block.items.empty())
                    html += "<p style='margin-top:0; margin-bottom:12px;'>"
                          + esc(block.items.front()) + "</p>";
                break;

            case BlockType::BulletList:
            case BlockType::OrderedList:
            {
                const char* tag = block.type == BlockType::BulletList ? "ul" : "ol";
                html += QString("<%1 style='margin-top:4px; margin-bottom:10px;'>").arg(tag);
                for (const auto& item : block.items)
                    html += "<li style='margin-bottom:2px;'>" + esc(item) + "</li>";
                html += QString("</%1>").arg(tag);
                break;
            }

            case BlockType::Note:
            {
                const bool warn = block.note_level == NoteLevel::Warning;
                const char* bg     = warn ? "#fdecea" : "#eef3ff";
                const char* border = warn ? "#e0b4b0" : "#b8c6e8";
                const char* label  = warn ? "Warning: " : "Note: ";
                const std::string txt = block.items.empty() ? std::string() : block.items.front();
                html += QString("<div style='margin:6px 0; padding:6px 10px; "
                                "background:%1; border:1px solid %2;'>"
                                "<b>%3</b>%4</div>")
                            .arg(bg).arg(border).arg(label).arg(esc(txt));
                break;
            }
        }
    }
    return html;
}
}

/**
 */
void SectionContentText::addContentUI(QVBoxLayout* layout,
                                      bool force_ui_reset)
{
    traced_assert(layout);

    if (isLocked())
    {
        layout->addWidget(lockStatePlaceholderWidget());
        return;
    }

    if (blocks_.empty())
        return;

    auto* view = new FittingTextView;
    view->setHtml(blocksToHtml(blocks_));
    layout->addWidget(view);
}

/**
 */
const std::vector<SectionContentText::Block>& SectionContentText::blocks() const
{
    return blocks_;
}

/**
 */
std::vector<std::string> SectionContentText::texts() const
{
    std::vector<std::string> out;
    for (const auto& block : blocks_)
    {
        switch (block.type)
        {
            case BlockType::Paragraph:
            case BlockType::Note:
                if (!block.items.empty())
                    out.push_back(block.items.front());
                break;
            case BlockType::BulletList:
            case BlockType::OrderedList:
                for (const auto& item : block.items)
                    out.push_back("- " + item);
                break;
        }
    }
    return out;
}

/**
 */
void SectionContentText::clearContent_impl()
{
    blocks_.clear();
}

namespace
{
// Serialize / deserialize the structured blocks.
nlohmann::json blocksToJSON(const std::vector<SectionContentText::Block>& blocks)
{
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& block : blocks)
    {
        nlohmann::json jb;
        jb["type"]  = blockTypeToString(block.type);
        jb["items"] = block.items;
        if (block.type == SectionContentText::BlockType::Note)
            jb["level"] = noteLevelToString(block.note_level);
        arr.push_back(std::move(jb));
    }
    return arr;
}

std::vector<SectionContentText::Block> blocksFromJSON(const nlohmann::json& arr)
{
    std::vector<SectionContentText::Block> blocks;
    if (!arr.is_array())
        return blocks;

    for (const auto& jb : arr)
    {
        SectionContentText::Block block;
        block.type = blockTypeFromString(jb.value("type", std::string("paragraph")));
        if (jb.contains("items") && jb.at("items").is_array())
            block.items = jb.at("items").get<std::vector<std::string>>();
        if (block.type == SectionContentText::BlockType::Note)
            block.note_level = noteLevelFromString(jb.value("level", std::string("info")));
        blocks.push_back(std::move(block));
    }
    return blocks;
}
}

/**
 */
void SectionContentText::toJSON_impl(nlohmann::json& j) const
{
    //call base
    SectionContent::toJSON_impl(j);

    j[ FieldBlocks ] = blocksToJSON(blocks_);
    j[ FieldTexts ]  = texts(); // legacy flat view for backward compatibility
}

/**
 */
bool SectionContentText::fromJSON_impl(const nlohmann::json& j)
{
    //call base
    if (!SectionContent::fromJSON_impl(j))
        return false;

    if (!j.is_object())
    {
        logerr << "section content text does not obtain needed fields";
        return false;
    }

    if (j.contains(FieldBlocks))
    {
        blocks_ = blocksFromJSON(j.at(FieldBlocks));
        return true;
    }

    // legacy report without structured blocks: parse the flat "texts" array,
    // honoring the historic '- ' bullet convention.
    if (j.contains(FieldTexts))
    {
        blocks_ = blocksFromLegacyTexts(j.at(FieldTexts).get<std::vector<std::string>>());
        return true;
    }

    logerr << "section content text does not obtain needed fields";
    return false;
}

/**
 */
Result SectionContentText::toJSONDocument_impl(nlohmann::json& j,
                                               const std::string* resource_dir,
                                               ReportExportMode export_style) const
{
    //call base
    auto r = SectionContent::toJSONDocument_impl(j, resource_dir, export_style);
    if (!r.ok())
        return r;

    j[ FieldBlocks ] = blocksToJSON(blocks_);
    j[ FieldTexts ]  = texts();

    return Result::succeeded();
}

}
