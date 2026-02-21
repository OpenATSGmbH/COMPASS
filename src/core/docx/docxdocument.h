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

#include "docxcontent.h"

#include <string>
#include <vector>
#include <map>

class DocxSection;

struct DocxImageEntry
{
    std::string source_path; // absolute path to the image file on disk
    std::string media_name;  // name inside word/media/ (e.g. "image1.png")
    std::string rel_id;      // relationship ID (e.g. "rId4")
    int width_emu  = 0;
    int height_emu = 0;
};

class DocxDocument : public DocxContent
{
public:
    DocxDocument(const std::string& path, const std::string& filename);

    void write();

    std::string title() const;
    void title(const std::string& title);

    std::string author() const;
    void author(const std::string& author);

    std::string abstract() const;
    void abstract(const std::string& abstract);

    void footerLeft(const std::string& text);
    void setLogo(const std::string& logo_path);

    std::string footerRelId() const;
    bool hasFooter() const;

    // hierarchical section access: "Foo:Bar:Baz" creates Foo → Bar → Baz
    DocxSection& getSection(const std::string& id);

    bool hasSubSection(const std::string& heading);
    DocxSection& getSubSection(const std::string& heading);
    void addSubSection(const std::string& heading);

    // register an image file — returns the relationship ID
    std::string addImageFile(const std::string& source_path);

    std::string path() const;
    std::string filename() const;

protected:
    std::string path_;
    std::string filename_;

    std::string title_;
    std::string author_;
    std::string abstract_;

    std::string footer_left_;   // left footer text (e.g. licensee)
    std::string logo_path_; // logo image path (used in footer and title page)

    std::vector<DocxImageEntry> images_;
    int next_rel_id_ = 5; // rId1=styles, rId2=footer, rId3=reserved; images start at rId5+

    std::string footer_rel_id_;       // relationship ID for footer part
    std::string footer_logo_rel_id_;  // relationship ID for logo image inside footer
    std::string title_logo_rel_id_;   // relationship ID for logo on title page (document body)

    std::string generateContentTypes() const;
    std::string generateRels() const;
    std::string generateDocumentRels() const;
    std::string generateFooterRels() const;
    std::string generateStyles() const;
    std::string generateDocumentXml();
    std::string generateFooterXml() const;

    std::string sectionPropertiesXml(bool landscape) const;

    void addZipEntry(struct archive* a, const std::string& name, const std::string& data) const;
    void addZipFile(struct archive* a, const std::string& archive_name, const std::string& disk_path) const;

    static std::string xmlEscape(const std::string& s);
};
