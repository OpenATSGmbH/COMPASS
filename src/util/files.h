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

#include <QDir>
#include <QIcon>

#include <map>
#include <string>

#include "global.h"

class QColor;

static const std::string SYSTEM_INSTALL_PATH = CMAKE_INSTALL_PREFIX + std::string("/compass/");

static const std::string HOME_PATH = QDir::homePath().toStdString();

static const std::string HOME_SUBDIRECTORY = HOME_PATH + "/.compass/";
static const std::string OSGEARTH_CACHE_SUBDIRECTORY = HOME_SUBDIRECTORY + "osgearth_cache/";
static const std::string HOME_VERSION_SUBDIRECTORY = HOME_SUBDIRECTORY + VERSION + "/";

static const std::string CONF_SUBDIRECTORY   = "conf/";
static const std::string DATA_SUBDIRECTORY   = "data/";
static const std::string PRESET_SUBDIRECTORY = "presets/";

static const std::string HOME_CONF_DIRECTORY   = HOME_VERSION_SUBDIRECTORY + CONF_SUBDIRECTORY;
static const std::string HOME_DATA_DIRECTORY   = HOME_VERSION_SUBDIRECTORY + DATA_SUBDIRECTORY;
static const std::string HOME_PRESET_DIRECTORY = HOME_VERSION_SUBDIRECTORY + PRESET_SUBDIRECTORY;

static const std::string LICENSE_SUBDIRECTORY = HOME_SUBDIRECTORY + "license/";

extern std::string CURRENT_CONF_DIRECTORY;

namespace Utils
{
namespace Files
{
bool fileExists(const std::string& path);
size_t fileSize(const std::string& path);
std::string fileSizeString(size_t file_size_in_bytes);
void verifyFileExists(const std::string& path);
bool directoryExists(const std::string& path);
bool copyRecursively(const std::string& source_folder, const std::string& dest_folder);
QStringList getFilesInDirectory(const std::string& path);
QStringList getSubdirectories(const std::string& path);

std::string getIconFilepath(const std::string& filename, bool verify = true);
std::string getImageFilepath(const std::string& filename, bool verify = true);
QIcon getIcon(const std::string& name, const QColor& color = QColor());

void deleteFile(const std::string& filename);
void deleteFolder(const std::string& path);

bool moveFile(const std::string& fp_old, const std::string& fp_new); // only works for existing folders

std::string getDirectoryFromPath(const std::string& path);
std::string getFilenameFromPath(const std::string& path);
bool createMissingDirectories(const std::string& path); // true if successful

std::string replaceExtension(const std::string& path, const std::string& new_ext_plus_point);

std::string normalizeFilename(const std::string& filename_without_ext, bool remove_special_chars);

std::string join(const std::string& path0, const std::string& path1);

class IconProvider {
public:
    static QIcon getIcon(const std::string& name);
};

}  // namespace Files
}  // namespace Utils
