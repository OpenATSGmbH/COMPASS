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

// Windows build feasibility test: osgEarth globe in an osgViewer window (GL core profile).
// Loads an earth file, renders for some seconds so that the tiles can load, reads back
// the frame buffer, writes osgearth_test.png and checks that a globe is visible.
// Exit code 0 means pass, 1 means fail.

#include <osgEarth/Common>
#include <osgEarth/MapNode>
#include <osgEarth/EarthManipulator>
#include <osgEarth/Version>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Image>
#include <osg/GL>
#include <osg/Version>

#include <gdal.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

namespace
{
struct CaptureCallback : public osg::Camera::DrawCallback
{
    mutable std::atomic<bool> request {false};
    mutable std::atomic<bool> done {false};
    mutable bool printed_gl {false};
    osg::ref_ptr<osg::Image> image {new osg::Image};

    void operator()(osg::RenderInfo& render_info) const override
    {
        if (!printed_gl)
        {
            std::printf("GL_VENDOR   %s\n", reinterpret_cast<const char*>(glGetString(GL_VENDOR)));
            std::printf("GL_RENDERER %s\n", reinterpret_cast<const char*>(glGetString(GL_RENDERER)));
            std::printf("GL_VERSION  %s\n", reinterpret_cast<const char*>(glGetString(GL_VERSION)));
            std::fflush(stdout);
            printed_gl = true;
        }

        if (!request || done)
            return;

        const osg::Viewport* viewport = render_info.getCurrentCamera()->getViewport();
        image->readPixels(static_cast<int>(viewport->x()), static_cast<int>(viewport->y()),
                          static_cast<int>(viewport->width()), static_cast<int>(viewport->height()),
                          GL_RGBA, GL_UNSIGNED_BYTE);
        done = true;
    }
};
}

int main(int argc, char** argv)
{
    const std::string earth_file = argc > 1 ? argv[1] : "test.earth";
    const double wait_seconds = argc > 2 ? std::stod(argv[2]) : 10.0;

    std::printf("OSG         %s\n", osgGetVersion());
    std::printf("osgEarth    %s\n", osgEarthGetVersion());
    std::printf("GDAL        %s\n", GDALVersionInfo("RELEASE_NAME"));
    std::fflush(stdout);

    osgEarth::initialize();

    osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(earth_file);
    if (!node.valid() || !osgEarth::MapNode::get(node.get()))
    {
        std::printf("ERROR could not load earth file %s\n", earth_file.c_str());
        std::printf("RESULT FAIL\n");
        return 1;
    }

    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
    viewer.setUpViewInWindow(50, 50, 800, 600);
    viewer.setCameraManipulator(new osgEarth::Util::EarthManipulator);
    viewer.getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
    viewer.setSceneData(node.get());

    osg::ref_ptr<CaptureCallback> capture = new CaptureCallback;
    viewer.getCamera()->setFinalDrawCallback(capture.get());

    viewer.realize();
    if (!viewer.isRealized())
    {
        std::printf("ERROR viewer could not be realized (no GL context)\n");
        std::printf("RESULT FAIL\n");
        return 1;
    }

    auto start = std::chrono::steady_clock::now();
    unsigned int frames = 0;

    while (!viewer.done() && !capture->done)
    {
        viewer.frame();
        ++frames;

        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        if (elapsed > wait_seconds)
            capture->request = true;

        if (elapsed > wait_seconds + 30.0)
            break;
    }

    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    std::printf("frames      %u in %.1f s (%.1f fps)\n", frames, elapsed, frames / elapsed);

    if (!capture->done || !capture->image->data())
    {
        std::printf("ERROR no frame buffer captured\n");
        std::printf("RESULT FAIL\n");
        return 1;
    }

    osg::Image* image = capture->image.get();
    osgDB::writeImageFile(*image, "osgearth_test.png");

    // globe in the center, empty space in the corner
    osg::Vec4 center = image->getColor(image->s() / 2, image->t() / 2);
    osg::Vec4 corner = image->getColor(2, 2);

    auto differs = [](const osg::Vec4& a, const osg::Vec4& b) {
        return std::abs(a.r() - b.r()) + std::abs(a.g() - b.g()) + std::abs(a.b() - b.b()) > 0.1f;
    };

    // share of pixels that differ from the background color (corner pixel)
    unsigned int globe = 0;
    unsigned int samples = 0;
    for (int y = 0; y < image->t(); y += 4)
        for (int x = 0; x < image->s(); x += 4)
        {
            ++samples;
            if (differs(image->getColor(x, y), corner))
                ++globe;
        }

    const double globe_share = static_cast<double>(globe) / samples;

    std::printf("pixel center %.2f %.2f %.2f\n", center.r(), center.g(), center.b());
    std::printf("pixel corner %.2f %.2f %.2f\n", corner.r(), corner.g(), corner.b());
    std::printf("globe share  %.2f\n", globe_share);

    const bool pass = differs(center, corner) && globe_share > 0.2 && globe_share < 0.95;

    std::printf("RESULT %s\n", pass ? "PASS" : "FAIL");
    std::fflush(stdout);

    return pass ? 0 : 1;
}
