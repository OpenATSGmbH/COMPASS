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

#include "asterixjsondecoder.h"
#include "asterixdecoderbase.h"
#include "asteriximporttask.h"
#include "util/files.h"

#include <jasterix/jasterix.h>

#include <fstream>
#include <sstream>

using namespace Utils;
using namespace std;
using namespace nlohmann;

/**
*/
class ASTERIXJSONReaderTextFile : public ASTERIXJSONReader
{
public:
    ASTERIXJSONReaderTextFile(size_t max_objects) : ASTERIXJSONReader(max_objects) {}
    virtual ~ASTERIXJSONReaderTextFile() = default;

    /**
    */
    bool open(const std::string& fn) override
    {
        close();

        if (fn.empty() || !Utils::Files::fileExists(fn))
            return false;

        file_stream_.open(fn);
        if (!file_stream_.is_open())
            return false;

        fn_ = fn;

        return true;
    }

    /**
    */
    void close() override
    {
        file_stream_.close();
        fn_ = "";
        open_count_ = 0;
        tmp_stream_.str("");
    }

    /**
    */
    bool readObjects(std::vector<std::string>& objects, bool& ok) override
    {
        objects.clear();
        ok = true;

        //stream already ended?
        if (file_stream_.eof())
        {
            loginf << "eof reached";
            return false;
        }

        const size_t MaxObjects = maxObjects();

        std::stringstream strm;

        // loop getting single characters and collect objects until max is reached
        char c;
        while (file_stream_.get(c) && objects.size() < MaxObjects)
        {
            bool closed_bracked = false;

            //check for brackets
            if (c == '{')
            {
                ++open_count_;
            }
            else if (c == '}')
            {
                --open_count_;
                closed_bracked = true;
            }

            // only add if enclosed by brackets
            if (open_count_ || closed_bracked)  
                tmp_stream_ << c;

            // next lines after objects
            if (c == '\n')  
                continue;

            //closed bracket encountered + balanced bracket count = full object retrieved
            if (closed_bracked && open_count_ == 0)
            {
                objects.push_back(tmp_stream_.str());
                tmp_stream_.str(""); //reset
            }
        }

        loginf << "parsed " << objects.size() << " object(s)";

        traced_assert(open_count_ == 0);  // nothing left open
        traced_assert(tmp_stream_.str().size() == 0 || tmp_stream_.str() == "\n");

        return !objects.empty();
    }

    /**
    */
    bool valid() const override
    {
        if (!fn_.empty() && file_stream_.is_open() && file_stream_)
            return true;

        return false;
    }

    /**
    */
    size_t numBytes() const override
    {
        return Utils::Files::fileSize(fn_);
    }

private:
    std::ifstream     file_stream_;
    std::stringstream tmp_stream_;
    std::string       fn_;

    size_t open_count_ = 0;
};

/**
*/
class ASTERIXJSONReaderArchive : public ASTERIXJSONReader
{
public:
    ASTERIXJSONReaderArchive(size_t max_objects) : ASTERIXJSONReader(max_objects) {}
    virtual ~ASTERIXJSONReaderArchive() = default;

    bool open(const std::string& fn) override
    {
        return false;
    }

    void close() override
    {
    }

    bool readObjects(std::vector<std::string>& objects, bool& ok) override
    {
        ok = false;
        return false;
    }

    bool valid() const override
    {
        return false;
    }

    size_t numBytes() const override
    {
        return 0;
    }

private:
};

/**
 * @param source Import source to retrieve data from.
 * @param settings If set, external settings will be applied, otherwise settings will be retrieved from the import task.
*/
ASTERIXJSONDecoder::ASTERIXJSONDecoder(ASTERIXImportSource& source,
                                       const ASTERIXImportTaskSettings* settings)
:   ASTERIXDecoderFile(ASTERIXImportSource::SourceType::FileJSON, source, settings)
{
}

/**
*/
ASTERIXJSONDecoder::~ASTERIXJSONDecoder() = default;

/**
*/
std::unique_ptr<ASTERIXJSONReader> ASTERIXJSONDecoder::readerForFile(const ASTERIXImportFileInfo& file_info, size_t max_objects) const
{
    if (ASTERIXDecoderFile::isSupportedArchive(file_info))
        return std::unique_ptr<ASTERIXJSONReader>(new ASTERIXJSONReaderArchive(max_objects));
    
    return std::unique_ptr<ASTERIXJSONReader>(new ASTERIXJSONReaderTextFile(max_objects));
}

/**
*/
std::unique_ptr<nlohmann::json> ASTERIXJSONDecoder::parseObjects(const std::vector<std::string>& objects,
                                                                 size_t& num_frames,
                                                                 size_t& num_records,
                                                                 size_t& num_errors) const
{
    std::unique_ptr<nlohmann::json> data(new nlohmann::json);

    num_frames  = 0;
    num_records = 0;
    num_errors  = 0;

    if (objects.empty())
        return data;

    *data = json::parse(objects.at(0));

    bool has_data_blocks = data->contains("data_blocks");
    bool has_frames      = data->contains("frames");
    bool is_jasterix     = (has_data_blocks || has_frames);

    if (is_jasterix)
    {
        traced_assert(objects.size() == 1);

        //@TODO: in case of jASTERIX json we actually read the whole file,
        //maybe we could split this in chunks?

        try
        {
            if (has_data_blocks) // no framing
            {
                logdbg << "data blocks found";

                traced_assert(data->at("data_blocks").is_array());

                std::vector<std::string> keys{"content", "records"};

                for (json& data_block : data->at("data_blocks"))
                {
                    if (!data_block.contains("category"))
                    {
                        logwrn << "data block without asterix category";
                        continue;
                    }
                    ++num_records;
                }
            }
            else // framed
            {
                logdbg << "no data blocks found, framed";

                traced_assert(has_frames);
                traced_assert(data->at("frames").is_array());

                std::vector<std::string> keys{"content", "records"};

                for (json& frame : data->at("frames"))
                {
                    if (!frame.contains("content"))  // frame with errors
                        continue;

                    traced_assert(frame.at("content").is_object());

                    if (!frame.at("content").contains("data_blocks"))  // frame with errors
                        continue;

                    traced_assert(frame.at("content").at("data_blocks").is_array());

                    for (json& data_block : frame.at("content").at("data_blocks"))
                    {
                        if (!data_block.contains("category"))  // data block with errors
                        {
                            logwrn << "data block without asterix category";
                            continue;
                        }
                        ++num_records;
                    }

                    ++num_frames;
                }
            }
        }
        catch (nlohmann::detail::parse_error& e)
        {
            logwrn << "jASTERIX: parse error " << e.what() << " in '" << data->at(0) << "'";
            ++num_errors;
        }
    }
    else
    {
        *data = nlohmann::json();
        (*data)["data"] = json::array();

        json& records = data->at("data");

        for (auto& str_it : objects)
        {
            try
            {
                records.push_back(json::parse(str_it));
                ++num_records;
            }
            catch (nlohmann::detail::parse_error& e)
            {
                logwrn << "parse error " << e.what() << " in '" << str_it << "'";
                ++num_errors;
            }
        }
    }

    return data;
}

/**
*/
void ASTERIXJSONDecoder::stop_impl()
{
    //@TODO
}

/**
*/
bool ASTERIXJSONDecoder::checkFile(ASTERIXImportFileInfo& file_info, std::string& error) const
{
    error = "";

    //already has error?
    if (file_info.hasError())
    {   
        error = "File info error encountered";
        return false;
    }

    //get reader
    auto reader = readerForFile(file_info, MaxJSONObjects);
    traced_assert(reader);

    //open file in reader
    if (!reader->open(file_info.filename))
    {
        error = "Could not open file";
        return false;
    }

    //check if opened file is valid
    if (!reader->valid())
    {
        error = "File invalid";
        return false;
    }

    //set custom size
    file_info.total_size_bytes = reader->numBytes();

    reader->close();

    return true;
}

/**
*/
bool ASTERIXJSONDecoder::checkDecoding(ASTERIXImportFileInfo& file_info, int section_idx, std::string& information, std::string& error) const
{
    //for now just assume that the files contain valid json
    return true;
}

/**
*/
void ASTERIXJSONDecoder::processFile(ASTERIXImportFileInfo& file_info)
{
    //get reader
    auto reader = readerForFile(file_info, MaxJSONObjects);
    traced_assert(reader);

    //open file in reader
    bool could_open_for_read = reader->open(file_info.filename);
    traced_assert(could_open_for_read);
    traced_assert(reader->valid());

    //read objects from file
    std::vector<std::string> objects;
    bool read_ok;

    auto checkReader = [ & ] ()
    {
        if (!read_ok)
        {
            logerr << "could not read JSON objects";
            logError("Could not read JSON objects");
            return false;
        }
        return true;
    };

    while (reader->readObjects(objects, read_ok))
    {
        if (!checkReader())
            break;

        //parse objects
        size_t num_frames;
        size_t num_records;
        size_t num_errors;

        auto data = parseObjects(objects, num_frames, num_records, num_errors);
        if (!data)
        {
            logerr << "could not parse JSON objects";
            logError("Could not parse JSON objects");
            break;
        }

        //push to job
        if (job() && !job()->obsolete())
            job()->fileJasterixCallback(std::move(data), settings().file_line_id_, num_frames, num_records, num_errors);

        size_t total_bytes = 0;
        for (const auto& obj : objects)
            total_bytes += obj.size();

        addChunkBytesRead(total_bytes);
        addRecordsRead(objects.size());
    }

    //final reader check
    checkReader();

    //reading ended
    reader->close();
}
