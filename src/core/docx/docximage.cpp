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

#include "docximage.h"

#include <sstream>

DocxImage::DocxImage(const std::string& filename, const std::string& caption,
                     const std::string& rel_id, int width_emu, int height_emu)
:   filename_ (filename)
,   caption_  (caption)
,   rel_id_   (rel_id)
,   width_emu_(width_emu)
,   height_emu_(height_emu)
{
}

std::string DocxImage::filename() const
{
    return filename_;
}

std::string DocxImage::relId() const
{
    return rel_id_;
}

std::string DocxImage::toXml()
{
    std::stringstream ss;

    std::string w = std::to_string(width_emu_);
    std::string h = std::to_string(height_emu_);

    // drawing paragraph with inline image
    ss << "<w:p>"
       << "<w:r>"
       << "<w:drawing>"
       << "<wp:inline distT=\"0\" distB=\"0\" distL=\"0\" distR=\"0\">"
       << "<wp:extent cx=\"" << w << "\" cy=\"" << h << "\"/>"
       << "<wp:docPr id=\"1\" name=\"" << caption_ << "\"/>"
       << "<a:graphic xmlns:a=\"http://schemas.openxmlformats.org/drawingml/2006/main\">"
       << "<a:graphicData uri=\"http://schemas.openxmlformats.org/drawingml/2006/picture\">"
       << "<pic:pic xmlns:pic=\"http://schemas.openxmlformats.org/drawingml/2006/picture\">"
       << "<pic:nvPicPr>"
       << "<pic:cNvPr id=\"0\" name=\"" << caption_ << "\"/>"
       << "<pic:cNvPicPr/>"
       << "</pic:nvPicPr>"
       << "<pic:blipFill>"
       << "<a:blip r:embed=\"" << rel_id_ << "\" xmlns:r=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships\"/>"
       << "<a:stretch><a:fillRect/></a:stretch>"
       << "</pic:blipFill>"
       << "<pic:spPr>"
       << "<a:xfrm>"
       << "<a:off x=\"0\" y=\"0\"/>"
       << "<a:ext cx=\"" << w << "\" cy=\"" << h << "\"/>"
       << "</a:xfrm>"
       << "<a:prstGeom prst=\"rect\"><a:avLst/></a:prstGeom>"
       << "</pic:spPr>"
       << "</pic:pic>"
       << "</a:graphicData>"
       << "</a:graphic>"
       << "</wp:inline>"
       << "</w:drawing>"
       << "</w:r>"
       << "</w:p>\n";

    // caption paragraph (italic, smaller)
    if (!caption_.empty())
    {
        std::string escaped;
        for (char c : caption_)
        {
            switch (c)
            {
                case '&':  escaped += "&amp;";  break;
                case '<':  escaped += "&lt;";   break;
                case '>':  escaped += "&gt;";   break;
                case '"':  escaped += "&quot;";  break;
                default:   escaped += c;        break;
            }
        }

        ss << "<w:p>"
           << "<w:pPr><w:jc w:val=\"center\"/></w:pPr>"
           << "<w:r><w:rPr><w:i/><w:sz w:val=\"20\"/></w:rPr>"
           << "<w:t xml:space=\"preserve\">" << escaped << "</w:t>"
           << "</w:r></w:p>\n";
    }

    return ss.str();
}
