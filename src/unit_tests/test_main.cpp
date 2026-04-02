#define CATCH_CONFIG_RUNNER
#include "catch.hpp"
#include <QApplication>
#include <QLocale>
#include <clocale>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    // Force C locale *after* QApplication ctor (which may reset it from env)
    // — matches what Client/MainWindow do in the real application.
    std::setlocale(LC_ALL, "C");
    QLocale::setDefault(QLocale::c());

    return Catch::Session().run(argc, argv);
}
