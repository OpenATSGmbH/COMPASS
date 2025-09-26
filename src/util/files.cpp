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

#include "files.h"
#include "logger.h"

#include <QDir>
#include <QFileInfo>
#include <QString>
#include <QRegularExpression>
#include <QApplication>
#include <QStyle>

#include "traced_assert.h"
#include <iostream>
#include <stdexcept>

#include <boost/filesystem.hpp>

std::string CURRENT_CONF_DIRECTORY;

namespace Utils
{
namespace Files
{
bool fileExists(const std::string& path)
{
    QFileInfo check_file(QString::fromStdString(path));
    // check if file exists and if yes: Is it really a file and no directory?
    return check_file.exists() && check_file.isFile();
}

size_t fileSize(const std::string& path)
{
    verifyFileExists(path);

    return boost::filesystem::file_size(path);
}

std::string fileSizeString(size_t file_size_in_bytes)
{
    double size = file_size_in_bytes;
    std::string unit = "";
    int dec = 0;

    if (size >= 1e12)
    {
        size /= 1e12;
        unit = "TB";
        dec = 1;
    }
    else if (size >= 1e09)
    {
        size /= 1e09;
        unit = "GB";
        dec = 1;
    }
    else if (size >= 1e06)
    {
        size /= 1e06;
        unit = "MB";
    }
    else if (size >= 1e03)
    {
        size /= 1e03;
        unit = "KB";
    }
    else
    {
        size /= 1e03;
        unit = "KB";
        dec = 1;
    }

    return QString::number(size, 'f', dec).toStdString() + " " + unit;
}

void verifyFileExists(const std::string& path)
{
    if (!fileExists(path))
        throw std::runtime_error("Utils: Files: verifyFileExists: file '" + path +
                                 "' does not exist");
}

bool directoryExists(const std::string& path)
{
    QFileInfo check_file(QString::fromStdString(path));
    // check if file exists and if yes: Is it really a directory?
    return check_file.exists() && check_file.isDir();
}

bool copyRecursively(const std::string& source_folder, const std::string& dest_folder)
{
    QString sourceFolder(QString::fromStdString(source_folder));
    QString destFolder(QString::fromStdString(dest_folder));
    bool success = false;
    QDir sourceDir(sourceFolder);

    if (!sourceDir.exists())
    {
        std::cout << "Files: copyRecursively: source dir " << source_folder << " doesn't exist"
                  << std::endl;
        return false;
    }

    QDir destDir(destFolder);
    if (!destDir.exists())
        destDir.mkdir(destFolder);

    QStringList files = sourceDir.entryList(QDir::Files);
    for (int i = 0; i < files.count(); i++)
    {
        QString srcName = sourceFolder + QDir::separator() + files[i];
        QString destName = destFolder + QDir::separator() + files[i];

        if (QFile::exists(destName))
            QFile::remove(destName);

        success = QFile::copy(srcName, destName);
        if (!success)
        {
            std::cout << "Files: copyRecursively: file copy failed of " << srcName.toStdString()
                      << " to " << destName.toStdString() << std::endl;
            return false;
        }
    }

    files.clear();
    files = sourceDir.entryList(QDir::AllDirs | QDir::NoDotAndDotDot);
    for (int i = 0; i < files.count(); i++)
    {
        QString srcName = sourceFolder + QDir::separator() + files[i];
        QString destName = destFolder + QDir::separator() + files[i];
        success = copyRecursively(srcName.toStdString(), destName.toStdString());
        if (!success)
        {
            std::cout << "Files: copyRecursively: directory copy failed of "
                      << srcName.toStdString() << " to " << destName.toStdString() << std::endl;
            return false;
        }
    }

    return true;
}

QStringList getFilesInDirectory(const std::string& path)
{
    traced_assert(directoryExists(path));
    QDir directory(path.c_str());
    QStringList list =
        directory.entryList(QStringList({"*"}), QDir::Files);  // << "*.jpg" << "*.JPG"
    return list;
}

QStringList getSubdirectories(const std::string& path)
{
    traced_assert(directoryExists(path));
    QDir directory(path.c_str());
    QStringList list =
        directory.entryList(QStringList({"*"}), QDir::Dirs | QDir::NoSymLinks | QDir::NoDotAndDotDot);
    return list;
}

std::string getIconFilepath(const std::string& filename, bool verify)
{
    std::string filepath = HOME_DATA_DIRECTORY + "icons/" + filename;
    if (verify)
        verifyFileExists(filepath);
    return filepath;
}

std::string getImageFilepath(const std::string& filename, bool verify)
{
    std::string filepath = HOME_DATA_DIRECTORY + "images/" + filename;
    if (verify)
        verifyFileExists(filepath);
    return filepath;
}

void deleteFile(const std::string& filename)
{
    QFile file(filename.c_str());
    file.remove();
}

void deleteFolder(const std::string& path)
{
    QDir dir(path.c_str());
    dir.removeRecursively();
}

bool moveFile(const std::string& fp_old, const std::string& fp_new)
{
    if (rename(fp_old.c_str(), fp_new.c_str()) != 0)
    {
        logerr << "renaming file from '" << fp_old << "' to '" << fp_new << "' failed with error " << std::strerror(errno);

        return false;
    }
    return true;
}

std::string getDirectoryFromPath(const std::string& path)
{
    boost::filesystem::path p(path);
    boost::filesystem::path dir = p.parent_path();

    return dir.string();
}

std::string getFilenameFromPath(const std::string& path)
{
    boost::filesystem::path p(path);
    boost::filesystem::path file = p.filename();

    return file.string();
}

bool createMissingDirectories(const std::string& path)
{
    QDir dir = QDir::root();
    bool ret = dir.mkpath(path.c_str());
    return ret;
}

std::string replaceExtension(const std::string& path, 
                             const std::string& new_ext_plus_point)
{
    boost::filesystem::path p(path);
    return p.replace_extension(new_ext_plus_point).string();
}

std::string normalizeFilename(const std::string& filename_without_ext, bool remove_special_chars)
{
    QString fn_lower = QString::fromStdString(filename_without_ext).toLower();

    if (remove_special_chars)
    {
        //replace non-letter-non-number with spaces (to be removed in the next step)
        for (int i = 0; i < fn_lower.count(); ++i)
            if (!fn_lower[ i ].isLetterOrNumber())
                fn_lower[ i ] = ' ';
    }

    fn_lower.remove(QRegularExpression("^[/\\s]+"));      //remove unwanted chars at begin
    fn_lower.remove(QRegularExpression("[/\\s]+$"));      //remove unwanted chars at end
    fn_lower.replace(QRegularExpression("[/\\s]+"), "_"); //replace sequences of unwanted chars with _
    
    return fn_lower.toStdString();
}

QIcon getIcon(const std::string& name, const QColor& color)
{
    QString path = getIconFilepath(name).c_str();
    if (!color.isValid())
        return QIcon(path);

    QImage img(path);
    if (img.isNull())
        return QIcon();

    img = img.convertToFormat(QImage::Format_ARGB32);

    int w = img.width();
    int h = img.height();

    const int r = color.red();
    const int g = color.green();
    const int b = color.blue();

    for (int y = 0; y < h; ++y)
    {
        auto line = img.scanLine(y);
        QRgb* pixel = reinterpret_cast<QRgb*>(line);
        
        for (int x = 0; x < w; ++x)
        {
            QRgb& px = pixel[ x ];

            int alpha = qAlpha(px);
            if (alpha == 0) 
                continue;

            int gray = 255 - qRed(px);

            int newR = (gray * r) / 255;
            int newG = (gray * g) / 255;
            int newB = (gray * b) / 255;

            px = qRgba(newR, newG, newB, alpha);
        }
    }
    
    return QIcon(QPixmap::fromImage(img));
}

QIcon IconProvider::getIcon(const std::string& name)
{
    static std::map<std::string, QIcon> icon_cache_;

    if (!icon_cache_.count(name))
    {
        QIcon icon;
        QString path = getIconFilepath(name).c_str();

        icon.addFile(path, QSize(QApplication::style()->pixelMetric(QStyle::PM_ToolBarIconSize),
                                 QApplication::style()->pixelMetric(QStyle::PM_ToolBarIconSize)));
        icon.addFile(path, QSize(QApplication::style()->pixelMetric(QStyle::PM_SmallIconSize),
                                 QApplication::style()->pixelMetric(QStyle::PM_SmallIconSize)));
        icon.addFile(path, QSize(QApplication::style()->pixelMetric(QStyle::PM_LargeIconSize),
                                 QApplication::style()->pixelMetric(QStyle::PM_LargeIconSize)));

        icon_cache_[name] = icon;
    }

    return icon_cache_.at(name);
}

}  // namespace Files
}  // namespace Utils
