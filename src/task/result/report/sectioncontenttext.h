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

#include "task/result/report/sectioncontent.h"

#include <vector>
#include <string>

class TaskManager;

namespace ResultReport
{

class Section;

/**
 * Prose content of a report section. Holds an ordered list of typed blocks
 * (paragraphs, bullet / numbered lists, note callouts) so that the Qt GUI,
 * the LaTeX/PDF export and the DocX export all render the same structure
 * natively, instead of each parsing a string convention.
 */
class SectionContentText : public SectionContent
{
public:
    enum class BlockType
    {
        Paragraph = 0,
        BulletList,
        OrderedList,
        Note
    };

    enum class NoteLevel
    {
        Info = 0,
        Warning
    };

    // One renderable block. A Paragraph / Note holds exactly one entry in
    // items; a BulletList / OrderedList holds one entry per item.
    struct Block
    {
        BlockType type = BlockType::Paragraph;
        NoteLevel note_level = NoteLevel::Info; // only meaningful for Note
        std::vector<std::string> items;
    };

    SectionContentText(unsigned int id,
                       const std::string& name,
                       Section* parent_section);
    SectionContentText(Section* parent_section);

    // Append one or more paragraphs. The string is split on '\n'; each
    // non-empty line becomes its own paragraph. No list markup is interpreted.
    void addText (const std::string& text);

    // Append a bullet list.
    void addList (const std::vector<std::string>& items);

    // Append a numbered list.
    void addOrderedList (const std::vector<std::string>& items);

    // Append a note / caveat callout.
    void addNote (const std::string& text, NoteLevel level = NoteLevel::Info);

    virtual std::string resourceExtension() const override;

    virtual void addContentUI(QVBoxLayout* layout,
                              bool force_ui_reset) override;

    const std::vector<Block>& blocks() const;

    // Flattened plain-text view (paragraphs / notes as-is, list items prefixed
    // with "- "). Used for the legacy "texts" JSON field and as a fallback for
    // any plain-text consumer.
    std::vector<std::string> texts() const;

    static const std::string FieldTexts;
    static const std::string FieldBlocks;

protected:
    void clearContent_impl() override final;

    void toJSON_impl(nlohmann::json& j) const override final;
    bool fromJSON_impl(const nlohmann::json& j) override final;
    Result toJSONDocument_impl(nlohmann::json& j,
                               const std::string* resource_dir,
                               ReportExportMode export_style) const override final;

    std::vector<Block> blocks_;
};

}
