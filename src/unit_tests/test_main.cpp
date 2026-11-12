#define CATCH_CONFIG_RUNNER
#include "catch.hpp"
#include "files.h"
#include "global.h"
#include "traced_assert.h"

#if USE_EXPERIMENTAL_SOURCE == true
#include "probimmplugin.h"
#include "docxplugin.h"
#include "extraanalysisplugin.h"
#endif

#include <QApplication>
#include <QLocale>
#include <clocale>

int main(int argc, char* argv[])
{
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
