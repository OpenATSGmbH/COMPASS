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

// File paths with non-ASCII characters. COMPASS passes paths as UTF-8 std::string to the
// narrow-string file functions (std::fstream, fopen, boost::filesystem). On Linux these take
// UTF-8. On Windows they use the active code page, which the application manifest sets to
// UTF-8 (src/app/windows/compass.manifest) - without it, the names on disk differ.
// Qt (QDir, QFileInfo) uses the wide-string functions on Windows and is the reference.

#include "catch.hpp"
#include "files.h"

#include <QDir>
#include <QFileInfo>
#include <QString>
#include <QStringList>

#include <boost/filesystem.hpp>

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>

namespace
{
// u8 literals are char8_t in C++20, plain strings with explicit UTF-8 bytes stay char
const std::string DirName  = "compass_utf8_test_\xC3\xA4\xC3\xB6\xC3\xBC\xC3\x9F";  // äöüß
const std::string FileName = "datei_\xC3\x84\xE2\x82\xAC.txt";                     // Ä€
const std::string Content  = "content \xC3\xA9";                                   // é
}

TEST_CASE("File paths with non-ASCII characters", "[files][utf8]")
{
    const std::string dir_path =
        (boost::filesystem::temp_directory_path() / DirName).generic_string();
    const std::string file_path = dir_path + "/" + FileName;

    QDir(QString::fromStdString(dir_path)).removeRecursively();

    // create with boost::filesystem, write with std::ofstream
    REQUIRE(boost::filesystem::create_directories(dir_path));
    {
        std::ofstream out(file_path, std::ios::binary);
        REQUIRE(out.is_open());
        out << Content;
    }

    // the names on disk, read with Qt
    const QStringList dirs = QDir(QString::fromStdString(
        boost::filesystem::temp_directory_path().generic_string())).entryList(QDir::Dirs);
    CHECK(dirs.contains(QString::fromStdString(DirName)));

    const QStringList files = QDir(QString::fromStdString(dir_path)).entryList(QDir::Files);
    REQUIRE(files.size() == 1);
    CHECK(files.front() == QString::fromStdString(FileName));

    // existence and size through all path functions
    CHECK(QFileInfo(QString::fromStdString(file_path)).exists());
    CHECK(Utils::Files::fileExists(file_path));
    CHECK(boost::filesystem::exists(file_path));
    CHECK(Utils::Files::fileSize(file_path) == Content.size());

    // read back with std::ifstream and fopen
    {
        std::ifstream in(file_path, std::ios::binary);
        REQUIRE(in.is_open());
        std::string read((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        CHECK(read == Content);
    }
    {
        FILE* fp = std::fopen(file_path.c_str(), "rb");
        REQUIRE(fp);
        char buf[64] = {0};
        size_t n = std::fread(buf, 1, sizeof(buf), fp);
        std::fclose(fp);
        CHECK(std::string(buf, n) == Content);
    }

    REQUIRE(QDir(QString::fromStdString(dir_path)).removeRecursively());
    CHECK_FALSE(boost::filesystem::exists(dir_path));
}
