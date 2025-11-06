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

#include "asterixfiledecoder.h"
#include "asterixdecoderbase.h"
#include "asteriximporttask.h"
#include "util/files.h"

#include <jasterix/jasterix.h>

using namespace Utils;
using namespace std;
using namespace nlohmann;


/**
 * @param source Import source to retrieve data from.
 * @param settings If set, external settings will be applied, otherwise settings will be retrieved from the import task.
*/
ASTERIXFileDecoder::ASTERIXFileDecoder(ASTERIXImportSource& source,
                                       const ASTERIXImportTaskSettings* settings)
:   ASTERIXDecoderFile(ASTERIXImportSource::SourceType::FileASTERIX, source, settings)
{
}

/**
*/
ASTERIXFileDecoder::~ASTERIXFileDecoder() = default;

/**
*/
void ASTERIXFileDecoder::stop_impl()
{
    // stop decoding
    task().jASTERIX()->stopFileDecoding();
}

/**
*/
bool ASTERIXFileDecoder::checkDecoding(ASTERIXImportFileInfo& file_info, 
                                       int section_idx, 
                                       std::string& information, 
                                       std::string& error,
                                       std::string& warning) const
{
    error   = "";
    warning = "";

    //get a fresh jasterix instance
    auto jasterix = task().jASTERIX(true);

    bool has_framing = settings().activeFileFraming().size() > 0;

    loginf << "file '" << file_info.filename << "' decoding now...";

    //analyze asterix file
    std::unique_ptr<nlohmann::json> analysis_info;
    analysis_info = has_framing ? jasterix->analyzeFile(file_info.filename, settings().activeFileFraming(), DecodeCheckRecordLimit) :
                                  jasterix->analyzeFile(file_info.filename, DecodeCheckRecordLimit);
    traced_assert(analysis_info);
    traced_assert(analysis_info->is_object());

    auto& file_error = file_info.error;

    //store analysis info for later usage
    file_error.analysis_info = *analysis_info;

    loginf << "file '" << file_info.filename << "' json '" << file_error.analysis_info.dump(4) << "'";
    //            json '{
    //               "data_items": {},
    //               "num_errors": 12,
    //               "num_records": 919,
    //               "sensor_counts": {}
    //           }'

    //no error info? => strange
    if (!file_error.analysis_info.contains("num_errors") ||
        !file_error.analysis_info.contains("num_records"))
    {
        error = "Decoding failed";
        return false;
    }

    //decoding succeeded?
    unsigned int num_errors  = file_error.analysis_info.at("num_errors");
    unsigned int num_records = file_error.analysis_info.at("num_records");

    if (num_errors) // decoder errors
    {
        error = "Decoding failed";
        return false;
    }

    if (!num_records) // no data
    {
        error = "Decoding failed";
        return false;
    }

    bool has_no_sac_sic          = false;
    bool has_no_sac_sic_critical = false;
    int  has_no_sac_sic_cat      = -1;
    bool has_invalid_cat         = false;
    bool has_no_sac_sic_item     = false;

    std::set<std::string> categories;
    for (const auto& sac_sic : analysis_info->items())
    {
        //no sensor => continue
        if (!sac_sic.value().is_object())
            continue;

        //check if there is a valid sac sic
        bool has_sac_sic = sac_sic.key() != "unknown";

        //unknown sac sic key => mark as error
        if (!has_sac_sic)
        {
            has_no_sac_sic = true;

            //no items to proof this is not cat001 or cat002? => mark as critical
            if (sac_sic.value().size() == 0)
                has_no_sac_sic_critical = true;
        }

        for (const auto& category : sac_sic.value().items())
        {
            //convert cat string to number
            bool cat_ok;
            auto cat = QString::fromStdString(category.key()).toInt(&cat_ok);

            if (cat_ok)
            {
                auto cat_str = String::categoryString(cat);
                categories.insert(cat_str);
            }
            else
            {
                //no valid cat string? => mark as error
                has_invalid_cat = true;
            }

            // no sac sic and cat cannot be proven to be cat001 or cat002 => mark as critical
            if (!has_sac_sic && (!cat_ok || (cat != 1 && cat != 2)))
            {
                has_no_sac_sic_critical = true;

                //remember any valid cat for error message
                if (cat_ok)
                    has_no_sac_sic_cat = cat;
            }

            // should have sac sic and no sac sic items found => mark as error
            if (has_sac_sic &&
                (!category.value().contains("010.SAC") ||
                 !category.value().contains("010.SIC")))
                has_no_sac_sic_item = true;
        }
    }

    //compile information string
    for (const auto& cat : categories)
        information += (information.empty() ? "" : ", ") + cat;

    //store errors
    if (has_no_sac_sic && has_no_sac_sic_critical)
    {
        error = "Missing SAC/SIC" + (has_no_sac_sic_cat >= 0 ? " in CAT" + String::categoryString((unsigned int)has_no_sac_sic_cat) : "");
    }
    else if (has_invalid_cat)
    {
        error = "Invalid category";
    }
    else if (has_no_sac_sic_item) // only makes sense if there was a sac sic
    {
        error = "No SAC/SIC data items found";
    }

    //store warnings
    if (has_no_sac_sic && !has_no_sac_sic_critical)
    {
        warning = "Missing SAC/SIC in CAT001 or CAT002";
    }
    
    return error.empty();
}

/**
*/
void ASTERIXFileDecoder::processFile(ASTERIXImportFileInfo& file_info)
{
    //get a fresh jasterix instance
    task().jASTERIX(true);

    string       current_filename  = file_info.filename;
    unsigned int current_file_line = settings().file_line_id_; //files_info_.at(current_file_count_).line_id_;

    loginf << "file '" << current_filename
           << "' framing '" << settings().activeFileFraming() << "' line " << current_file_line;

    //jasterix callback
    auto callback = [this, current_file_line, &file_info] (std::unique_ptr<nlohmann::json> data, 
                                               size_t num_frames,
                                               size_t num_records, 
                                               size_t num_errors) 
    {
        // get last index

        if (settings().activeFileFraming() == "")
        {
            traced_assert(data->contains("data_blocks"));
            traced_assert(data->at("data_blocks").is_array());

            if (data->at("data_blocks").size())
            {
                json& data_block = data->at("data_blocks").back();

                traced_assert(data_block.contains("content"));
                traced_assert(data_block.at("content").is_object());
                traced_assert(data_block.at("content").contains("index"));

                setFileBytesRead(data_block.at("content").at("index"));
            }
        }
        else
        {
            traced_assert(data->contains("frames"));
            traced_assert(data->at("frames").is_array());

            if (data->at("frames").size())
            {
                json& frame = data->at("frames").back();

                if (frame.contains("content"))
                {
                    traced_assert(frame.at("content").is_object());
                    traced_assert(frame.at("content").contains("index"));

                    setFileBytesRead(frame.at("content").at("index"));
                }
            }
        }

        addRecordsRead(num_records);

        if (num_errors)
        {
            file_info.error.errtype = ASTERIXImportFileError::ErrorType::DecodingFailed;
            file_info.error.errinfo = "Decoding errors: "+to_string(num_errors);
        }

        //invoke job callback
        if (job() && !job()->obsolete())
            job()->fileJasterixCallback(std::move(data), current_file_line, num_frames, num_records, num_errors);
    };

    //start decoding
    if (settings().activeFileFraming() == "")
        task().jASTERIX()->decodeFile(current_filename, callback);
    else
        task().jASTERIX()->decodeFile(current_filename, settings().activeFileFraming(), callback);
}
