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

#include <boost/date_time/posix_time/posix_time.hpp>

/**
*/
class ASTERIXFileDecoder : public ASTERIXDecoderFile
{
public:
    ASTERIXFileDecoder(ASTERIXImportSource& source,
                       const ASTERIXImportTaskSettings* settings = nullptr);
    virtual ~ASTERIXFileDecoder();

    std::string name() const override final { return "ASTERIXFileDecoder"; }

protected:
    void stop_impl() override final;

    bool checkDecoding(ASTERIXImportFileInfo& file_info, 
                       int section_idx, 
                       std::string& information, 
                       std::string& error,
                       std::string& warning) const override final;
    void processFile(ASTERIXImportFileInfo& file_info) override final;
};
