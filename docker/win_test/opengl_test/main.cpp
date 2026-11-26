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

// Windows build feasibility test: Qt widget with an OpenGL 3.3 core context.
// Prints the OpenGL driver strings, draws a red triangle on a blue background,
// reads back the frame buffer, checks two pixels and quits with 0 (pass) or 1 (fail).

#include <QApplication>
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QSurfaceFormat>
#include <QImage>
#include <QTimer>

#include <cstdio>
#include <thread>
#include <mutex>

namespace
{
const char* vertex_shader_src = R"(
#version 330 core
layout(location = 0) in vec2 pos;
void main() { gl_Position = vec4(pos, 0.0, 1.0); }
)";

const char* fragment_shader_src = R"(
#version 330 core
out vec4 color;
void main() { color = vec4(1.0, 0.0, 0.0, 1.0); }
)";
}

class TestWidget : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
public:
    bool gl_ok_ {false};

protected:
    void initializeGL() override
    {
        if (!initializeOpenGLFunctions())
        {
            std::printf("ERROR could not resolve OpenGL 3.3 core functions\n");
            return;
        }

        std::printf("GL_VENDOR   %s\n", reinterpret_cast<const char*>(glGetString(GL_VENDOR)));
        std::printf("GL_RENDERER %s\n", reinterpret_cast<const char*>(glGetString(GL_RENDERER)));
        std::printf("GL_VERSION  %s\n", reinterpret_cast<const char*>(glGetString(GL_VERSION)));
        std::printf("GLSL        %s\n", reinterpret_cast<const char*>(glGetString(GL_SHADING_LANGUAGE_VERSION)));

        QSurfaceFormat fmt = context()->format();
        std::printf("context     %d.%d profile %s\n", fmt.majorVersion(), fmt.minorVersion(),
                    fmt.profile() == QSurfaceFormat::CoreProfile ? "core" : "compatibility/none");

        if (!program_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_shader_src)
            || !program_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment_shader_src)
            || !program_.link())
        {
            std::printf("ERROR shader program %s\n", program_.log().toStdString().c_str());
            return;
        }

        const float vertices[] = {-0.8f, -0.8f, 0.8f, -0.8f, 0.0f, 0.8f};

        vao_.create();
        vao_.bind();
        vbo_.create();
        vbo_.bind();
        vbo_.allocate(vertices, sizeof(vertices));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        vao_.release();

        gl_ok_ = true;
    }

    void paintGL() override
    {
        glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        if (!gl_ok_)
            return;

        program_.bind();
        vao_.bind();
        glDrawArrays(GL_TRIANGLES, 0, 3);
        vao_.release();
        program_.release();
    }

private:
    QOpenGLShaderProgram     program_;
    QOpenGLBuffer            vbo_;
    QOpenGLVertexArrayObject vao_;
};

int main(int argc, char** argv)
{
    // check the posix thread model of the toolchain
    {
        std::mutex mutex;
        int value = 0;
        std::thread worker([&] { std::lock_guard<std::mutex> lock(mutex); value = 42; });
        worker.join();
        std::printf("std::thread %s\n", value == 42 ? "ok" : "FAILED");
    }

    QCoreApplication::setAttribute(Qt::AA_UseDesktopOpenGL);

    QSurfaceFormat format;
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication app(argc, argv);

    TestWidget widget;
    widget.resize(400, 400);
    widget.setWindowTitle("COMPASS Windows OpenGL Test");
    widget.show();

    QTimer::singleShot(1500, [&] {
        QImage image = widget.grabFramebuffer();
        image.save("opengl_test.png");

        QColor center = image.pixelColor(image.width() / 2, image.height() / 2);
        QColor corner = image.pixelColor(5, 5);

        std::printf("pixel center %d %d %d (expected red)\n", center.red(), center.green(), center.blue());
        std::printf("pixel corner %d %d %d (expected blue)\n", corner.red(), corner.green(), corner.blue());

        bool pass = widget.gl_ok_
                    && center.red() > 200 && center.green() < 50 && center.blue() < 50
                    && corner.red() < 50 && corner.green() < 50 && corner.blue() > 200;

        std::printf("RESULT %s\n", pass ? "PASS" : "FAIL");
        std::fflush(stdout);

        app.exit(pass ? 0 : 1);
    });

    return app.exec();
}
