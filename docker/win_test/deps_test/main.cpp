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

// Windows build test: calls one function of each COMPASS dependency and prints the result.
// Exit code 0 means all checks passed.

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/program_options.hpp>
#include <boost/regex.hpp>
#include <boost/stacktrace.hpp>
#include <boost/thread.hpp>

#include <Eigen/Dense>

#include <log4cpp/Category.hh>
#include <log4cpp/OstreamAppender.hh>
#include <log4cpp/PatternLayout.hh>

#include <gdal.h>
#include <archive.h>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>
#include <tbb/version.h>
#include <openssl/crypto.h>
#include <openssl/sha.h>
#include <nmeaparse/nmea.h>
#include <GeographicLib/Geodesic.hpp>
#include <duckdb.h>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

namespace
{
int failures = 0;

void check(const std::string& name, bool ok, const std::string& info)
{
    std::printf("%-12s %-4s %s\n", name.c_str(), ok ? "ok" : "FAIL", info.c_str());
    std::fflush(stdout);
    if (!ok)
        ++failures;
}
}

int main(int argc, char** argv)
{
    // Boost
    {
        boost::program_options::options_description desc("test");
        desc.add_options()("help", "help");
        boost::program_options::variables_map vm;
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);

        boost::regex re("CAT(\\d{3})");
        boost::smatch match;
        std::string text = "ASTERIX CAT048";
        bool regex_ok = boost::regex_search(text, match, re) && match[1] == "048";

        std::ostringstream compressed;
        {
            boost::iostreams::filtering_ostream out;
            out.push(boost::iostreams::gzip_compressor());
            out.push(compressed);
            out << "COMPASS gzip test";
        }
        std::istringstream compressed_in(compressed.str());
        boost::iostreams::filtering_istream in;
        in.push(boost::iostreams::gzip_decompressor());
        in.push(compressed_in);
        std::string line;
        std::getline(in, line);

        int thread_value = 0;
        boost::thread worker([&] { thread_value = 42; });
        worker.join();

        auto now = boost::posix_time::microsec_clock::local_time();

        check("boost", regex_ok && line == "COMPASS gzip test" && thread_value == 42 && !now.is_not_a_date_time(),
              "regex, iostreams gzip, thread, posix_time, cwd "
              + boost::filesystem::current_path().string());

        boost::stacktrace::stacktrace trace;
        std::ostringstream trace_text;
        trace_text << trace;
        std::string first_frame = trace_text.str().substr(0, trace_text.str().find('\n'));
        check("stacktrace", trace.size() > 0, std::to_string(trace.size()) + " frames, first: " + first_frame);
    }

    // Eigen
    {
        Eigen::Matrix3d m;
        m << 2, 0, 0, 0, 3, 0, 0, 0, 4;
        Eigen::Matrix3d product = m * m.inverse();
        check("eigen", product.isIdentity(1e-12), "matrix inverse");
    }

    // log4cpp
    {
        auto* appender = new log4cpp::OstreamAppender("console", &std::cout);
        auto* layout = new log4cpp::PatternLayout();
        layout->setConversionPattern("             log4cpp line: %p %m%n");
        appender->setLayout(layout);
        log4cpp::Category& root = log4cpp::Category::getRoot();
        root.addAppender(appender);
        root.setPriority(log4cpp::Priority::INFO);
        root.info("hello from log4cpp");
        std::cout.flush();
        check("log4cpp", true, "root category written");
    }

    // GDAL
    {
        GDALAllRegister();
        check("gdal", GDALGetDriverCount() > 0,
              std::string(GDALVersionInfo("RELEASE_NAME")) + ", " + std::to_string(GDALGetDriverCount()) + " drivers");
    }

    // libarchive
    {
        struct archive* a = archive_read_new();
        bool ok = a && archive_read_support_filter_bzip2(a) == ARCHIVE_OK
                  && archive_read_support_filter_gzip(a) == ARCHIVE_OK
                  && archive_read_support_format_zip(a) == ARCHIVE_OK;
        if (a)
            archive_read_free(a);
        check("libarchive", ok, std::string(archive_version_details()));
    }

    // TBB (long long: long is 32 bit on Windows)
    {
        long long sum = tbb::parallel_reduce(
            tbb::blocked_range<long long>(0, 1000000), 0LL,
            [](const tbb::blocked_range<long long>& r, long long init) {
                for (long long i = r.begin(); i != r.end(); ++i)
                    init += i;
                return init;
            },
            [](long long a, long long b) { return a + b; });
        check("tbb", sum == 499999500000LL, std::string("runtime ") + TBB_runtime_version());
    }

    // OpenSSL
    {
        unsigned char digest[SHA256_DIGEST_LENGTH];
        const char* data = "abc";
        SHA256(reinterpret_cast<const unsigned char*>(data), 3, digest);
        // SHA-256("abc") starts with ba7816bf
        bool ok = digest[0] == 0xba && digest[1] == 0x78 && digest[2] == 0x16 && digest[3] == 0xbf;
        check("openssl", ok, OpenSSL_version(OPENSSL_VERSION));
    }

    // NemaTode
    {
        nmea::NMEAParser parser;
        nmea::GPSService gps(parser);
        bool ok = true;
        try
        {
            parser.readLine("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
        }
        catch (const std::exception&)
        {
            ok = false;
        }
        ok = ok && std::abs(gps.fix.latitude - 48.1173) < 0.001;
        check("nematode", ok, "GGA latitude " + std::to_string(gps.fix.latitude));
    }

    // GeographicLib
    {
        double s12 = 0;
        // Vienna to Salzburg
        GeographicLib::Geodesic::WGS84().Inverse(48.2082, 16.3738, 47.8095, 13.0550, s12);
        check("geographic", s12 > 240000 && s12 < 260000, "distance Vienna-Salzburg " + std::to_string(s12 / 1000) + " km");
    }

    // DuckDB (C API)
    {
        duckdb_database db;
        duckdb_connection con;
        duckdb_result result;
        bool ok = duckdb_open(nullptr, &db) == DuckDBSuccess;
        ok = ok && duckdb_connect(db, &con) == DuckDBSuccess;
        ok = ok && duckdb_query(con, "SELECT 40 + 2", &result) == DuckDBSuccess;
        int64_t value = ok ? duckdb_value_int64(&result, 0, 0) : 0;
        if (ok)
        {
            duckdb_destroy_result(&result);
            duckdb_disconnect(&con);
            duckdb_close(&db);
        }
        check("duckdb", ok && value == 42, std::string(duckdb_library_version()) + ", SELECT 40 + 2 = " + std::to_string(value));
    }

    std::printf("RESULT %s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
