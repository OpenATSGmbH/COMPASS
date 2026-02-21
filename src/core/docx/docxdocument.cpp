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

#include "docxdocument.h"
#include "docxsection.h"

#include <archive.h>
#include <archive_entry.h>

#include <QImage>
#include <QDateTime>

#include <sstream>
#include <fstream>
#include <stdexcept>
#include <cassert>

#include <boost/filesystem.hpp>

DocxDocument::DocxDocument(const std::string& path, const std::string& filename)
:   path_    (path)
,   filename_(filename)
{
}

std::string DocxDocument::title() const { return title_; }
void DocxDocument::title(const std::string& title) { title_ = title; }

std::string DocxDocument::author() const { return author_; }
void DocxDocument::author(const std::string& author) { author_ = author; }

std::string DocxDocument::abstract() const { return abstract_; }
void DocxDocument::abstract(const std::string& abstract) { abstract_ = abstract; }

void DocxDocument::footerLeft(const std::string& text) { footer_left_ = text; }

void DocxDocument::footerRightLogo(const std::string& logo_path)
{
    footer_right_logo_path_ = logo_path;
    footer_rel_id_      = "rId3"; // document → footer1.xml
    footer_logo_rel_id_ = "rId1"; // footer → logo image
}

std::string DocxDocument::footerRelId() const { return footer_rel_id_; }

bool DocxDocument::hasFooter() const
{
    return !footer_left_.empty() || !footer_right_logo_path_.empty();
}

std::string DocxDocument::path() const { return path_; }
std::string DocxDocument::filename() const { return filename_; }

std::string DocxDocument::xmlEscape(const std::string& s)
{
    std::string out;
    out.reserve(s.size());
    for (char c : s)
    {
        switch (c)
        {
            case '&':  out += "&amp;";  break;
            case '<':  out += "&lt;";   break;
            case '>':  out += "&gt;";   break;
            case '"':  out += "&quot;"; break;
            case '\'': out += "&apos;"; break;
            default:   out += c;        break;
        }
    }
    return out;
}

DocxSection& DocxDocument::getSection(const std::string& id)
{
    // split by ':'
    std::vector<std::string> parts;
    std::stringstream ss(id);
    std::string part;
    while (std::getline(ss, part, ':'))
    {
        if (!part.empty())
            parts.push_back(part);
    }

    assert(!parts.empty());

    // navigate/create hierarchy
    if (!hasSubSection(parts[0]))
        addSubSection(parts[0]);

    DocxSection* current = &getSubSection(parts[0]);

    for (size_t i = 1; i < parts.size(); ++i)
    {
        if (!current->hasSubSection(parts[i]))
            current->addSubSection(parts[i]);
        current = &current->getSubSection(parts[i]);
    }

    return *current;
}

bool DocxDocument::hasSubSection(const std::string& heading)
{
    return findSubSection(heading) != nullptr;
}

DocxSection& DocxDocument::getSubSection(const std::string& heading)
{
    auto* s = findSubSection(heading);
    assert(s);
    return *s;
}

void DocxDocument::addSubSection(const std::string& heading)
{
    assert(!hasSubSection(heading));
    sub_content_.push_back(std::make_unique<DocxSection>(1, heading));
}

std::string DocxDocument::addImageFile(const std::string& source_path)
{
    // check if already registered
    for (const auto& img : images_)
    {
        if (img.source_path == source_path)
            return img.rel_id;
    }

    DocxImageEntry entry;
    entry.source_path = source_path;

    auto ext = boost::filesystem::path(source_path).extension().string();
    entry.media_name = "image" + std::to_string(images_.size() + 1) + ext;
    entry.rel_id = "rId" + std::to_string(next_rel_id_++);

    // read image dimensions for EMU calculation
    QImage img(QString::fromStdString(source_path));
    if (!img.isNull())
    {
        // target: fit within ~16cm width, maintain aspect ratio
        const int max_width_emu = 5760000;  // ~16cm in EMU (1cm = 360000 EMU)

        double dpi_x = img.dotsPerMeterX() > 0 ? img.dotsPerMeterX() / 39.3701 : 96.0;
        double dpi_y = img.dotsPerMeterY() > 0 ? img.dotsPerMeterY() / 39.3701 : 96.0;

        int w_emu = (int)(img.width()  / dpi_x * 914400.0);
        int h_emu = (int)(img.height() / dpi_y * 914400.0);

        if (w_emu > max_width_emu)
        {
            double scale = (double)max_width_emu / w_emu;
            w_emu = max_width_emu;
            h_emu = (int)(h_emu * scale);
        }

        entry.width_emu  = w_emu;
        entry.height_emu = h_emu;
    }
    else
    {
        entry.width_emu  = 5760000; // 16cm default
        entry.height_emu = 3240000; // 9cm default
    }

    images_.push_back(entry);
    return entry.rel_id;
}

// --- XML part generators ---

std::string DocxDocument::generateContentTypes() const
{
    std::stringstream ss;
    ss << R"(<?xml version="1.0" encoding="UTF-8" standalone="yes"?>)"
       << R"(<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">)"
       << R"(<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>)"
       << R"(<Default Extension="xml" ContentType="application/xml"/>)"
       << R"(<Default Extension="png" ContentType="image/png"/>)"
       << R"(<Default Extension="jpg" ContentType="image/jpeg"/>)"
       << R"(<Default Extension="jpeg" ContentType="image/jpeg"/>)"
       << R"(<Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>)"
       << R"(<Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>)";

    if (hasFooter())
        ss << R"(<Override PartName="/word/footer1.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.footer+xml"/>)";

    ss << "</Types>";
    return ss.str();
}

std::string DocxDocument::generateRels() const
{
    return R"(<?xml version="1.0" encoding="UTF-8" standalone="yes"?>)"
           R"(<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">)"
           R"(<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>)"
           R"(</Relationships>)";
}

std::string DocxDocument::generateDocumentRels() const
{
    std::stringstream ss;
    ss << R"(<?xml version="1.0" encoding="UTF-8" standalone="yes"?>)"
       << R"(<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">)"
       << R"(<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/>)";

    if (hasFooter())
    {
        ss << R"(<Relationship Id=")" << footer_rel_id_
           << R"(" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/footer" Target="footer1.xml"/>)";
    }

    for (const auto& img : images_)
    {
        ss << R"(<Relationship Id=")" << img.rel_id
           << R"(" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="media/)"
           << img.media_name << R"("/>)";
    }

    ss << "</Relationships>";
    return ss.str();
}

std::string DocxDocument::generateStyles() const
{
    std::stringstream ss;
    ss << R"(<?xml version="1.0" encoding="UTF-8" standalone="yes"?>)"
       << R"(<w:styles xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">)";

    // default style
    ss << R"(<w:style w:type="paragraph" w:default="1" w:styleId="Normal">)"
       << R"(<w:name w:val="Normal"/>)"
       << R"(<w:rPr><w:sz w:val="22"/><w:szCs w:val="22"/></w:rPr>)"
       << "</w:style>";

    // heading styles 1-6
    const char* sizes[] = {"32", "28", "26", "24", "22", "20"};
    for (int i = 1; i <= 6; ++i)
    {
        ss << R"(<w:style w:type="paragraph" w:styleId="Heading)" << i << R"(">)"
           << R"(<w:name w:val="heading )" << i << R"("/>)"
           << R"(<w:basedOn w:val="Normal"/>)"
           << R"(<w:next w:val="Normal"/>)"
           << R"(<w:pPr><w:keepNext/><w:spacing w:before="240" w:after="120"/>)"
           << R"(<w:outlineLvl w:val=")" << (i - 1) << R"("/>)"
           << "</w:pPr>"
           << R"(<w:rPr><w:b/><w:sz w:val=")" << sizes[i - 1] << R"("/><w:szCs w:val=")" << sizes[i - 1] << R"("/>)</w:rPr>)"
           << "</w:style>";
    }

    // table of contents heading
    ss << R"(<w:style w:type="paragraph" w:styleId="TOCHeading">)"
       << R"(<w:name w:val="TOC Heading"/>)"
       << R"(<w:basedOn w:val="Heading1"/>)"
       << "</w:style>";

    ss << "</w:styles>";
    return ss.str();
}

std::string DocxDocument::generateDocumentXml()
{
    std::stringstream ss;
    ss << R"(<?xml version="1.0" encoding="UTF-8" standalone="yes"?>)"
       << R"(<w:document xmlns:wpc="http://schemas.microsoft.com/office/word/2010/wordprocessingCanvas")"
       << R"( xmlns:mo="http://schemas.microsoft.com/office/mac/office/2008/main")"
       << R"( xmlns:mc="http://schemas.openxmlformats.org/markup-compatibility/2006")"
       << R"( xmlns:mv="urn:schemas-microsoft-com:mac:vml")"
       << R"( xmlns:o="urn:schemas-microsoft-com:office:office")"
       << R"( xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships")"
       << R"( xmlns:m="http://schemas.openxmlformats.org/officeDocument/2006/math")"
       << R"( xmlns:v="urn:schemas-microsoft-com:vml")"
       << R"( xmlns:wp="http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing")"
       << R"( xmlns:w10="urn:schemas-microsoft-com:office:word")"
       << R"( xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main")"
       << R"( xmlns:wne="http://schemas.microsoft.com/office/word/2006/wordml")"
       << R"( xmlns:wp14="http://schemas.microsoft.com/office/word/2010/wordprocessingDrawing")"
       << R"( mc:Ignorable="w14 wp14">)";

    ss << "<w:body>\n";

    // title page
    if (!title_.empty())
    {
        ss << "<w:p><w:pPr>"
           << "<w:pStyle w:val=\"Heading1\"/>"
           << "<w:jc w:val=\"center\"/>"
           << "</w:pPr>"
           << "<w:r><w:rPr><w:sz w:val=\"48\"/><w:szCs w:val=\"48\"/></w:rPr>"
           << "<w:t xml:space=\"preserve\">" << xmlEscape(title_) << "</w:t>"
           << "</w:r></w:p>\n";

        if (!author_.empty())
        {
            ss << "<w:p><w:pPr><w:jc w:val=\"center\"/></w:pPr>"
               << "<w:r><w:rPr><w:sz w:val=\"24\"/></w:rPr>"
               << "<w:t xml:space=\"preserve\">" << xmlEscape(author_) << "</w:t>"
               << "</w:r></w:p>\n";
        }

        // date
        auto date = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm").toStdString();
        ss << "<w:p><w:pPr><w:jc w:val=\"center\"/></w:pPr>"
           << "<w:r><w:rPr><w:sz w:val=\"22\"/></w:rPr>"
           << "<w:t xml:space=\"preserve\">" << xmlEscape(date) << "</w:t>"
           << "</w:r></w:p>\n";

        if (!abstract_.empty())
        {
            // spacing before abstract
            ss << "<w:p/>\n";
            ss << "<w:p>"
               << "<w:r><w:rPr><w:i/></w:rPr>"
               << "<w:t xml:space=\"preserve\">" << xmlEscape(abstract_) << "</w:t>"
               << "</w:r></w:p>\n";
        }

        // page break after title
        ss << "<w:p><w:r><w:br w:type=\"page\"/></w:r></w:p>\n";
    }

    // all sections and content
    ss << DocxContent::toXml();

    // final section properties (page size A4 portrait)
    ss << sectionPropertiesXml(false) << "\n";

    ss << "</w:body></w:document>";
    return ss.str();
}

// --- ZIP writing via LibArchive ---

void DocxDocument::addZipEntry(struct archive* a, const std::string& name, const std::string& data) const
{
    auto entry = archive_entry_new();

    archive_entry_set_pathname(entry, name.c_str());
    archive_entry_set_size(entry, data.size());
    archive_entry_set_filetype(entry, AE_IFREG);
    archive_entry_set_perm(entry, 0644);

    archive_write_header(a, entry);
    archive_write_data(a, data.data(), data.size());

    archive_entry_free(entry);
}

void DocxDocument::addZipFile(struct archive* a, const std::string& archive_name,
                              const std::string& disk_path) const
{
    std::ifstream f(disk_path, std::ios::binary | std::ios::ate);
    if (!f.is_open())
        throw std::runtime_error("DocxDocument: cannot open image file: " + disk_path);

    auto size = f.tellg();
    f.seekg(0, std::ios::beg);

    std::vector<char> buf(size);
    f.read(buf.data(), size);

    auto entry = archive_entry_new();

    archive_entry_set_pathname(entry, archive_name.c_str());
    archive_entry_set_size(entry, size);
    archive_entry_set_filetype(entry, AE_IFREG);
    archive_entry_set_perm(entry, 0644);

    archive_write_header(a, entry);
    archive_write_data(a, buf.data(), buf.size());

    archive_entry_free(entry);
}

void DocxDocument::write()
{
    // ensure directory exists
    boost::filesystem::create_directories(path_);

    std::string full_path = (boost::filesystem::path(path_) / boost::filesystem::path(filename_)).string();

    auto a = archive_write_new();
    if (!a)
        throw std::runtime_error("DocxDocument: archive_write_new failed");

    archive_write_set_format_zip(a);

    if (archive_write_open_filename(a, full_path.c_str()) != ARCHIVE_OK)
    {
        std::string err = archive_error_string(a);
        archive_write_free(a);
        throw std::runtime_error("DocxDocument: cannot create " + full_path + ": " + err);
    }

    // write XML parts
    addZipEntry(a, "[Content_Types].xml",        generateContentTypes());
    addZipEntry(a, "_rels/.rels",                generateRels());
    addZipEntry(a, "word/document.xml",          generateDocumentXml());
    addZipEntry(a, "word/styles.xml",            generateStyles());
    addZipEntry(a, "word/_rels/document.xml.rels", generateDocumentRels());

    // write footer parts
    if (hasFooter())
    {
        addZipEntry(a, "word/footer1.xml", generateFooterXml());

        if (!footer_right_logo_path_.empty())
        {
            addZipEntry(a, "word/_rels/footer1.xml.rels", generateFooterRels());

            auto ext = boost::filesystem::path(footer_right_logo_path_).extension().string();
            addZipFile(a, "word/media/footer_logo" + ext, footer_right_logo_path_);
        }
    }

    // write image files
    for (const auto& img : images_)
    {
        addZipFile(a, "word/media/" + img.media_name, img.source_path);
    }

    archive_write_close(a);
    archive_write_free(a);
}

std::string DocxDocument::sectionPropertiesXml(bool landscape) const
{
    std::stringstream ss;
    ss << "<w:sectPr>";

    if (hasFooter())
        ss << R"(<w:footerReference w:type="default" r:id=")" << footer_rel_id_ << R"("/>)";

    if (landscape)
    {
        ss << R"(<w:pgSz w:w="16838" w:h="11906" w:orient="landscape"/>)"
           << R"(<w:pgMar w:top="1134" w:right="1134" w:bottom="1134" w:left="1134" w:header="709" w:footer="709" w:gutter="0"/>)";
    }
    else
    {
        ss << R"(<w:pgSz w:w="11906" w:h="16838"/>)"
           << R"(<w:pgMar w:top="1134" w:right="1134" w:bottom="1134" w:left="1134" w:header="709" w:footer="709" w:gutter="0"/>)";
    }

    ss << "</w:sectPr>";
    return ss.str();
}

std::string DocxDocument::generateFooterXml() const
{
    std::stringstream ss;
    ss << R"(<?xml version="1.0" encoding="UTF-8" standalone="yes"?>)"
       << R"(<w:ftr xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main")"
       << R"( xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships")"
       << R"( xmlns:wp="http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing")"
       << R"( xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main")"
       << R"( xmlns:pic="http://schemas.openxmlformats.org/drawingml/2006/picture">)";

    // single paragraph with tab stops: left | center | right
    ss << "<w:p><w:pPr>"
       << "<w:tabs>"
       << R"(<w:tab w:val="center" w:pos="4819"/>)"
       << R"(<w:tab w:val="right" w:pos="9638"/>)"
       << "</w:tabs>"
       << R"(<w:rPr><w:sz w:val="16"/><w:szCs w:val="16"/></w:rPr>)"
       << "</w:pPr>";

    // left: licensee text
    if (!footer_left_.empty())
    {
        ss << R"(<w:r><w:rPr><w:sz w:val="16"/><w:szCs w:val="16"/></w:rPr>)"
           << R"(<w:t xml:space="preserve">)" << xmlEscape(footer_left_) << "</w:t></w:r>";
    }

    // tab to center
    ss << "<w:r><w:tab/></w:r>";

    // center: page number field
    ss << R"(<w:fldSimple w:instr=" PAGE ">)"
       << R"(<w:r><w:rPr><w:sz w:val="16"/><w:szCs w:val="16"/></w:rPr>)"
       << "<w:t>1</w:t></w:r></w:fldSimple>";

    // tab to right
    ss << "<w:r><w:tab/></w:r>";

    // right: logo image
    if (!footer_right_logo_path_.empty())
    {
        // compute logo dimensions — target height ~0.5cm = 180000 EMU
        int logo_h_emu = 180000;
        int logo_w_emu = 180000; // default square

        QImage logo_img(QString::fromStdString(footer_right_logo_path_));
        if (!logo_img.isNull() && logo_img.height() > 0)
        {
            double aspect = (double)logo_img.width() / logo_img.height();
            logo_w_emu = (int)(logo_h_emu * aspect);
        }

        ss << "<w:r><w:drawing>"
           << R"(<wp:inline distT="0" distB="0" distL="0" distR="0">)"
           << R"(<wp:extent cx=")" << logo_w_emu << R"(" cy=")" << logo_h_emu << R"("/>)"
           << R"(<wp:docPr id="1000" name="Footer Logo"/>)"
           << "<a:graphic>"
           << R"(<a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/picture">)"
           << "<pic:pic>"
           << R"(<pic:nvPicPr><pic:cNvPr id="1000" name="footer_logo"/><pic:cNvPicPr/></pic:nvPicPr>)"
           << R"(<pic:blipFill><a:blip r:embed=")" << footer_logo_rel_id_ << R"("/>)"
           << "<a:stretch><a:fillRect/></a:stretch></pic:blipFill>"
           << "<pic:spPr>"
           << R"(<a:xfrm><a:off x="0" y="0"/><a:ext cx=")" << logo_w_emu << R"(" cy=")" << logo_h_emu << R"("/></a:xfrm>)"
           << R"(<a:prstGeom prst="rect"><a:avLst/></a:prstGeom>)"
           << "</pic:spPr>"
           << "</pic:pic>"
           << "</a:graphicData></a:graphic>"
           << "</wp:inline></w:drawing></w:r>";
    }

    ss << "</w:p></w:ftr>";
    return ss.str();
}

std::string DocxDocument::generateFooterRels() const
{
    std::stringstream ss;
    ss << R"(<?xml version="1.0" encoding="UTF-8" standalone="yes"?>)"
       << R"(<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">)";

    if (!footer_right_logo_path_.empty())
    {
        auto ext = boost::filesystem::path(footer_right_logo_path_).extension().string();
        ss << R"(<Relationship Id=")" << footer_logo_rel_id_
           << R"(" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="media/footer_logo)"
           << ext << R"("/>)";
    }

    ss << "</Relationships>";
    return ss.str();
}
