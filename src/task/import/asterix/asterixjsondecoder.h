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

#include "asterixdecoderfile.h"

#include <string>
#include <memory>

/**
*/
class ASTERIXJSONReader
{
public:
    ASTERIXJSONReader(size_t max_objects) : max_objects_(max_objects) {}
    virtual ~ASTERIXJSONReader() = default;

    virtual bool open(const std::string& fn) = 0;
    virtual void close() = 0;
    virtual bool readObjects(std::vector<std::string>& objects, bool& ok) = 0;
    virtual bool valid() const = 0;
    virtual size_t numBytes() const = 0;

    size_t maxObjects() const { return max_objects_; }

private:
    size_t max_objects_ = 0;
};

/**
*/
class ASTERIXJSONDecoder : public ASTERIXDecoderFile
{
public:
    ASTERIXJSONDecoder(ASTERIXImportSource& source,
                       const ASTERIXImportTaskSettings* settings = nullptr);
    virtual ~ASTERIXJSONDecoder();

protected:
    void stop_impl() override final;

    bool checkFile(ASTERIXImportFileInfo& file_info, std::string& error) const override final;
    bool checkDecoding(ASTERIXImportFileInfo& file_info, int section_idx, std::string& information, std::string& error) const override final;
    void processFile(ASTERIXImportFileInfo& file_info) override final;

private:
    std::unique_ptr<ASTERIXJSONReader> readerForFile(const ASTERIXImportFileInfo& file_info, size_t max_objects) const;
    std::unique_ptr<nlohmann::json> parseObjects(const std::vector<std::string>& objects,
                                                 size_t& num_frames,
                                                 size_t& num_records,
                                                 size_t& num_errors) const;
    void checkCAT001SacSics(nlohmann::json& data_block);
};
