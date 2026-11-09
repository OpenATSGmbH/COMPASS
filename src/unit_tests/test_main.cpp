#define CATCH_CONFIG_RUNNER
#include "catch.hpp"
#include "files.h"
#include "traced_assert.h"

#include <QApplication>
#include <QLocale>
#include <clocale>

int main(int argc, char* argv[])
{
    // Run without visible windows (and without a display, e.g. in CI) unless a platform
    // is requested explicitly, e.g. QT_QPA_PLATFORM=xcb to watch widget tests.
    if (!qEnvironmentVariableIsSet("QT_QPA_PLATFORM"))
        qputenv("QT_QPA_PLATFORM", "offscreen");

    QApplication app(argc, argv);

    // Force C locale *after* QApplication ctor (which may reset it from env)
    // - matches what Client/MainWindow do in the real application.
    std::setlocale(LC_ALL, "C");
    QLocale::setDefault(QLocale::c());

    // data resources (icons, maps, ...) from the repository instead of ~/.compass
    const std::string data_dir = std::string(COMPASS_SOURCE_DIR) + "/data/";
    traced_assert(Utils::Files::directoryExists(data_dir));
    Utils::Files::setDataDirectory(data_dir);

    return Catch::Session().run(argc, argv);
}
