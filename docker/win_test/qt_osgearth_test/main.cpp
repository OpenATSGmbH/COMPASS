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

// Windows build feasibility test: osgEarth map in the Geographic View ViewerWidget
// (QOpenGLWidget + GraphicsWindowQt from experimental_src, unchanged).
//
// Uses the OpenGL setup of client.cpp and the map loading and shutdown sequence of
// GeographicViewDataWidget and geo_view::shutdown(). Every shutdown step is printed
// with a time stamp, so that a hang at exit can be located.
//
// Usage: qt_osgearth_test.exe <earth file> [wait seconds] [--no-jobs-shutdown]

#include "ViewerWidget.h"

#include <osgEarth/MapNode>
#include <osgEarth/Map>
#include <osgEarth/Viewpoint>
#include <osgEarth/weejobs.h>
#include <osgDB/ReadFile>
#include <osgDB/DatabasePager>
#include <osgViewer/ViewerEventHandlers>
#include <osg/DisplaySettings>

#include <QApplication>
#include <QElapsedTimer>
#include <QImage>
#include <QMainWindow>
#include <QSurfaceFormat>
#include <QThread>
#include <QTimer>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

namespace
{
QElapsedTimer g_timer;

void step(const std::string& text)
{
    std::printf("[%7lld ms] %s\n", static_cast<long long>(g_timer.elapsed()), text.c_str());
    std::fflush(stdout);
}

// same as GeographicViewDataWidget::drainMapJobs()
void drainJobs()
{
    auto* metrics = jobs::get_metrics();
    const int timeout_ms = 5000;
    int waited = 0;

    while (metrics && waited < timeout_ms)
    {
        int pending = metrics->total_pending();
        int running = metrics->total_running();
        if (pending + running == 0)
            break;
        if (waited % 500 == 0)
            step("  draining osgEarth jobs: pending " + std::to_string(pending)
                 + " running " + std::to_string(running));
        QThread::msleep(10);
        waited += 10;
    }

    step(waited >= timeout_ms ? "  drain TIMEOUT" : "  drained after " + std::to_string(waited) + " ms");
}
}

int main(int argc, char** argv)
{
    g_timer.start();

    std::string earth_file;
    double wait_seconds = 10.0;
    bool jobs_shutdown = true;

    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--no-jobs-shutdown") == 0)
            jobs_shutdown = false;
        else if (earth_file.empty())
            earth_file = argv[i];
        else
            wait_seconds = std::stod(argv[i]);
    }

    if (earth_file.empty())
    {
        std::printf("usage: %s <earth file> [wait seconds] [--no-jobs-shutdown]\n", argv[0]);
        return 1;
    }

    // OpenGL setup as in client.cpp
    QSurfaceFormat surf_format = QSurfaceFormat::defaultFormat();
    surf_format.setVersion(3, 3);
    surf_format.setProfile(QSurfaceFormat::OpenGLContextProfile::CoreProfile);
    osg::DisplaySettings::instance()->setGLContextVersion("3.3");
    osg::DisplaySettings::instance()->setShaderHint(osg::DisplaySettings::SHADER_GL3);
    QSurfaceFormat::setDefaultFormat(surf_format);

    QApplication app(argc, argv);

    osgEarth::initialize();

    step("loading " + earth_file);

    osg::ref_ptr<osg::Node> loaded = osgDB::readNodeFile(earth_file);
    osg::ref_ptr<osgEarth::MapNode> map_node = osgEarth::MapNode::get(loaded.get());
    loaded = nullptr;

    if (!map_node.valid())
    {
        std::printf("ERROR could not load earth file\nRESULT FAIL\n");
        return 1;
    }

    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addChild(map_node.get());

    QMainWindow window;
    window.setWindowTitle("COMPASS Windows Qt osgEarth Test");

    auto* viewer_widget = new osgEarth::QtGui::ViewerWidget();
    window.setCentralWidget(viewer_widget);
    window.resize(1000, 700);

    osgViewer::Viewer* viewer = viewer_widget->getViewer();
    viewer->addEventHandler(new osgViewer::WindowSizeHandler);
    viewer->setSceneData(root.get());

    auto* manip = dynamic_cast<osgEarth::Util::EarthManipulator*>(viewer->getCameraManipulator());
    if (manip)
        manip->setViewpoint(osgEarth::Viewpoint("Austria", 14.0, 47.5, 0.0, 0.0, -90.0, 1.5e6));

    window.show();
    step("window shown");

    int result = 1;

    QTimer::singleShot(static_cast<int>(wait_seconds * 1000), [&]() {
        step("capture");

        // as the Geographic View export: drive one frame, then read the FBO
        viewer_widget->setExporting(true);
        viewer_widget->driveFrame();
        QImage image = viewer_widget->grabFramebuffer();
        viewer_widget->setExporting(false);

        image.save("qt_osgearth_test.png");

        // share of pixels that differ from the background color (corner pixel)
        QColor corner = image.pixelColor(2, 2);
        unsigned int content = 0;
        unsigned int samples = 0;
        for (int y = 0; y < image.height(); y += 4)
            for (int x = 0; x < image.width(); x += 4)
            {
                ++samples;
                QColor c = image.pixelColor(x, y);
                if (std::abs(c.red() - corner.red()) + std::abs(c.green() - corner.green())
                    + std::abs(c.blue() - corner.blue()) > 25)
                    ++content;
            }

        const double content_share = samples ? static_cast<double>(content) / samples : 0.0;

        std::printf("image        %dx%d\n", image.width(), image.height());
        std::printf("background   %d %d %d\n", corner.red(), corner.green(), corner.blue());
        std::printf("content share %.3f\n", content_share);

        const bool pass = content_share > 0.01 && content_share < 0.99;
        std::printf("RESULT %s\n", pass ? "PASS" : "FAIL");
        std::fflush(stdout);
        result = pass ? 0 : 1;

        // shutdown sequence as in ~GeographicViewDataWidget
        step("shutdown: viewer done, pager cancel");
        osg::ref_ptr<osgViewer::Viewer> viewer_keep_alive = viewer;
        viewer->setDone(true);
        if (osgDB::DatabasePager* pager = viewer->getDatabasePager())
        {
            pager->cancel();
            pager->clear();
        }

        step("shutdown: ViewerWidget::shutdown");
        viewer_widget->shutdown();
        root->removeChild(map_node.get());

        step("shutdown: remove layers");
        osgEarth::LayerVector layers;
        map_node->getMap()->getLayers(layers);
        for (auto& layer : layers)
            layer->setOpenAutomatically(false);
        for (auto& layer : layers)
            map_node->getMap()->removeLayer(layer.get());
        layers.clear();

        step("shutdown: drain jobs");
        drainJobs();

        step("shutdown: release map node");
        map_node = nullptr;
        root = nullptr;

        step("shutdown: delete viewer widget");
        window.takeCentralWidget();
        delete viewer_widget;
        viewer_widget = nullptr;
        viewer_keep_alive = nullptr;

        step("quit event loop");
        app.quit();
    });

    app.exec();
    step("event loop done");

    window.close();

    // as geo_view::shutdown()
    step("releaseRetiredContexts");
    osgEarth::QtGui::ViewerWidget::releaseRetiredContexts();

    if (jobs_shutdown)
    {
        step("jobs::shutdown");
        jobs::shutdown();
    }
    else
    {
        step("jobs::shutdown skipped");
    }

    step("main returns");
    return result;
}
