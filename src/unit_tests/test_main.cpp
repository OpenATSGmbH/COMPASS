#define CATCH_CONFIG_RUNNER
#include "catch.hpp"
#include "files.h"
#include "global.h"
#include "system.h"
#include "traced_assert.h"

#if USE_EXPERIMENTAL_SOURCE == true
#include "probimmplugin.h"
#include "docxplugin.h"
#include "extraanalysisplugin.h"
#endif

#ifdef _WIN32
#include <osgDB/Registry>
#endif

#include "exit_hooks.h"

#include <QApplication>
#include <QLocale>
#include <clocale>

int main(int argc, char* argv[])
{
    // stops e.g. the osgEarth job pool before the process ends (exit_hooks.h): on Windows the
    // process exit waits forever otherwise. Declared first, so it runs after all other locals.
    exit_hooks::Scope exit_hooks_scope;

    // Run without visible windows (and without a display, e.g. in CI) unless a platform
    // is requested explicitly, e.g. QT_QPA_PLATFORM=xcb to watch widget tests.
    if (!qEnvironmentVariableIsSet("QT_QPA_PLATFORM"))
    {
#ifdef _WIN32
        // offscreen has no font database on Windows (no text drawn at all), minimal with
        // enable_fonts uses the Windows fonts and shows no windows either
        qputenv("QT_QPA_PLATFORM", "minimal:enable_fonts");
#else
        qputenv("QT_QPA_PLATFORM", "offscreen");
#endif
    }

    QApplication app(argc, argv);

    // Force C locale *after* QApplication ctor (which may reset it from env)
    // - matches what Client/MainWindow do in the real application.
    std::setlocale(LC_ALL, "C");
    QLocale::setDefault(QLocale::c());

    // data resources (icons, maps, ...) from the repository instead of ~/.compass
    const std::string data_dir = std::string(COMPASS_SOURCE_DIR) + "/data/";
    traced_assert(Utils::Files::directoryExists(data_dir));
    Utils::Files::setDataDirectory(data_dir);

#ifdef _WIN32
    // OSG plugins (e.g. the osgEarth terrain engine) in lib/ of the deployment (win_deploy.sh --with_tests),
    // as in the client (client.cpp). On Linux OSG finds them in the system paths.
    osgDB::Registry::instance()->setLibraryFilePathList(std::string(Utils::System::appDir()) + "/lib");
#endif

    // same plugins as the client (main.cpp), core step only (factory entries)
#if USE_EXPERIMENTAL_SOURCE == true
    static ProbIMMPlugin probimm_plugin;
    probimm_plugin.initCore();

    static DocXPlugin docx_plugin;
    docx_plugin.initCore();

    static ExtraAnalysisPlugin extra_analysis_plugin;
    extra_analysis_plugin.initCore();
#endif

    return Catch::Session().run(argc, argv);
}
