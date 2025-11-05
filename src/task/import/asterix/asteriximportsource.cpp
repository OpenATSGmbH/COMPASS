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

#include "asteriximportsource.h"

#include "files.h"

/**
*/
nlohmann::json ASTERIXImportFileError::toJSON() const
{
    nlohmann::json info;

    info[ "errtype" ] = (int)errtype;
    info[ "errinfo" ] = errinfo;

    //info[ "analysis_info" ] = analysis_info;

    return info;
}

/**
*/
nlohmann::json ASTERIXImportFileSection::toJSON() const
{
    nlohmann::json info;

    info[ "id"               ] = id;
    info[ "idx"              ] = idx;
    info[ "description"      ] = description;
    info[ "raw_data_size"    ] = raw_data.size();
    info[ "total_size_bytes" ] = total_size_bytes;
    info[ "used"             ] = used;
    info[ "error"            ] = error.toJSON();

    return info;
}

/****************************************************************************************
 * ASTERIXImportFileInfo
 ****************************************************************************************/

/**
*/
void ASTERIXImportFileInfo::reset()
{
    error.reset();
    warning = "";
    sections.clear();

    decoding_tested = false;
    processed       = false;
    used            = true;
}

/**
*/
bool ASTERIXImportFileInfo::hasError() const 
{
    //file itself has an error?
    if (error.hasError())
        return true;

    //used file section has an error?
    for (const auto& s : sections)
        if (s.used && s.error.hasError())
            return true;

    return false;
}

/**
*/
bool ASTERIXImportFileInfo::hasWarning() const 
{
    //file itself has a warning?
    if (!warning.empty())
        return true;

    //used file section has an error?
    for (const auto& s : sections)
        if (s.used && !s.warning.empty())
            return true;

    return false;
}

/**
*/
bool ASTERIXImportFileInfo::canDecode() const
{
    //erroneous file => cannot decode
    if (hasError())
        return false;

    //no subsections marked for usage => cannot decode
    if (hasSections() && numUsedSections() == 0)
        return false;

    return true;
}

/**
*/
bool ASTERIXImportFileInfo::hasSections() const
{
    return !sections.empty();
}

/**
*/
size_t ASTERIXImportFileInfo::numUsedSections() const
{
    size_t num_used = 0;

    for (const auto& s : sections)
        if (s.used)
            ++num_used;

    return num_used;
}

/**
*/
size_t ASTERIXImportFileInfo::sizeInBytes(bool used_only) const
{
    //obtain import size from used sections?
    if (hasSections())
    {
        size_t size = 0;
        for (const auto& s : sections)
            if (!used_only || s.used)
                size += s.total_size_bytes;

        return size;
    }

    //check custom size value first
    if (total_size_bytes.has_value())
        return total_size_bytes.value();

    //determine import size from file size
    if (filename.empty() || !Utils::Files::fileExists(filename))
        return 0;

    return Utils::Files::fileSize(filename);
}

/**
*/
nlohmann::json ASTERIXImportFileInfo::toJSON() const
{
    nlohmann::json info;

    info[ "filename"        ] = filename;
    info[ "used"            ] = used;
    info[ "decoding_tested" ] = decoding_tested;
    info[ "error"           ] = error.toJSON();
    info[ "warning"         ] = warning;

    nlohmann::json sec_infos = nlohmann::json::array();
    for (const auto& s : sections)
        sec_infos.push_back(s.toJSON());

    info[ "sections" ] = sec_infos;

    return info;
}

/****************************************************************************************
 * ASTERIXImportFileInfo
 ****************************************************************************************/

/**
*/
ASTERIXImportSource::ASTERIXImportSource() = default;

/**
*/
ASTERIXImportSource::~ASTERIXImportSource() = default;

/**
*/
std::string ASTERIXImportSource::sourceTypeAsString(SourceType source_type)
{
    if (source_type == SourceType::FileASTERIX)
        return "File";
    else if (source_type == SourceType::FileJSON)
        return "JSON";
    else if (source_type == SourceType::FilePCAP)
        return "PCAP";
    else if (source_type == SourceType::NetASTERIX)
        return "Network";
    
    return "Unknown";
}

/**
*/
bool ASTERIXImportSource::isFileType(SourceType source_type)
{
    if (source_type == SourceType::FileASTERIX)
        return true;
    else if (source_type == SourceType::FileJSON)
        return true;
    else if (source_type == SourceType::FilePCAP)
        return true;
    else if (source_type == SourceType::NetASTERIX)
        return false;
    
    return false;
}

/**
*/
bool ASTERIXImportSource::isNetworkType(SourceType source_type)
{
    if (source_type == SourceType::FileASTERIX)
        return false;
    else if (source_type == SourceType::FileJSON)
        return false;
    else if (source_type == SourceType::FilePCAP)
        return false;
    else if (source_type == SourceType::NetASTERIX)
        return true;
    
    return false;
}

/**
*/
void ASTERIXImportSource::setSourceType(SourceType source)
{
    source_type_ = source;

    clear_internal();

    emit sourceChanged();
    emit filesChanged();
    emit changed();
}

/**
*/
void ASTERIXImportSource::setSourceType(SourceType source, const std::vector<std::string>& filenames)
{
    source_type_ = source;

    clear_internal();
    addFiles_internal(filenames);

    emit sourceChanged();
    emit filesChanged();
    emit changed();
}

/**
*/
void ASTERIXImportSource::clear_internal()
{
    file_infos_.clear();
}

/**
*/
void ASTERIXImportSource::addFile_internal(const std::string& fn)
{
    file_infos_.push_back(ASTERIXImportFileInfo());
    file_infos_.back().filename = fn;
}

/**
*/
void ASTERIXImportSource::addFiles_internal(const std::vector<std::string>& filenames)
{
    for (const std::string& fn : filenames)
        addFile_internal(fn);
}

/**
*/
void ASTERIXImportSource::addFile(const std::string& fn)
{
    addFile_internal(fn);

    emit filesChanged();
    emit changed();
}

/**
*/
void ASTERIXImportSource::addFiles(const std::vector<std::string>& filenames)
{
    addFiles_internal(filenames);

    emit filesChanged();
    emit changed();
}

/**
*/
void ASTERIXImportSource::clearFiles()
{
    clear_internal();

    emit filesChanged();
    emit changed();
}

/**
*/
std::string ASTERIXImportSource::filesAsString() const
{
    std::string ret;

    for (auto& file_info : file_infos_)
        ret += file_info.filename + "\n";

    return ret;
}

/**
*/
const ASTERIXImportSource::FileInfos& ASTERIXImportSource::files() const 
{ 
    return file_infos_; 
}

/**
*/
ASTERIXImportSource::SourceType ASTERIXImportSource::sourceType() const 
{ 
    return source_type_; 
}

/**
*/
std::string ASTERIXImportSource::sourceTypeAsString() const
{
    return ASTERIXImportSource::sourceTypeAsString(source_type_);
}

/**
*/
bool ASTERIXImportSource::isFileType() const
{
    return ASTERIXImportSource::isFileType(source_type_);
}

/**
*/
bool ASTERIXImportSource::isNetworkType() const
{
    return ASTERIXImportSource::isNetworkType(source_type_);
}

/**
*/
size_t ASTERIXImportSource::totalFileSizeInBytes(bool used_only) const
{
    size_t size = 0;

    for (const auto& fi : file_infos_)
        if (!used_only || fi.used)
            size += fi.sizeInBytes(used_only);

    return size;
}

/**
*/
nlohmann::json ASTERIXImportSource::toJSON() const
{
    nlohmann::json info;

    info[ "source_type" ] = (int)source_type_;

    nlohmann::json fi_infos = nlohmann::json::array();
    for (const auto& fi : file_infos_)
        fi_infos.push_back(fi.toJSON());

    info[ "file_infos"  ] = fi_infos;

    return info;
}

/**
*/
void ASTERIXImportSource::setFileUsage(bool use, 
                                       size_t file_idx, 
                                       int section_idx)
{
    auto& fi = file_infos_.at(file_idx);

    auto& usage = section_idx >= 0 ? fi.sections.at(section_idx).used : fi.used;

    if (usage != use)
    {
        usage = use;
        emit fileUsageChanged();
    }
}
