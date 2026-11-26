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

// Windows build test: draws text into images in several ways and counts the visible pixels.
// Investigates text missing in QWidget::render into a transparent QImage under Wine
// (test_styledlabel.cpp). Run once with the default (GDI) font engine and once with
// -platform windows:fontengine=freetype.

#include <QApplication>
#include <QImage>
#include <QLabel>
#include <QPainter>
#include <QFontDatabase>

#include <iostream>

namespace
{
int visiblePixels(const QImage& img, const QColor& background)
{
    const QImage argb = img.convertToFormat(QImage::Format_ARGB32);
    int n = 0;
    for (int y = 0; y < argb.height(); ++y)
        for (int x = 0; x < argb.width(); ++x)
            if (argb.pixel(x, y) != background.rgba())
                ++n;
    return n;
}

QImage drawText(QImage::Format format, const QColor& background, bool antialias)
{
    QImage img(300, 40, format);
    img.fill(background);

    QPainter painter(&img);
    QFont font = QApplication::font();
    if (!antialias)
        font.setStyleStrategy(QFont::NoAntialias);
    painter.setFont(font);
    painter.setPen(Qt::red);
    painter.drawText(img.rect(), Qt::AlignCenter, "DLH4AB 3412 FL350");

    return img;
}

QImage renderLabel(const QColor& background)
{
    QLabel label("DLH4AB 3412 FL350");
    label.resize(label.sizeHint());

    QImage img(label.size(), QImage::Format_ARGB32_Premultiplied);
    img.fill(background);
    label.render(&img, QPoint(), QRegion(), QWidget::DrawChildren);

    return img;
}
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    std::cout << "platform " << QApplication::platformName().toStdString()
              << ", font '" << QApplication::font().family().toStdString() << "' "
              << QApplication::font().pointSizeF() << " pt" << std::endl;

    const QColor transparent(Qt::transparent);
    const QColor white(Qt::white);

    struct Case { const char* name; QImage img; QColor bg; };
    const Case cases[] = {
        { "drawText ARGB32_Premultiplied transparent", drawText(QImage::Format_ARGB32_Premultiplied, transparent, true), transparent },
        { "drawText ARGB32 transparent              ", drawText(QImage::Format_ARGB32, transparent, true), transparent },
        { "drawText ARGB32_Premultiplied white      ", drawText(QImage::Format_ARGB32_Premultiplied, white, true), white },
        { "drawText RGB32 white                     ", drawText(QImage::Format_RGB32, white, true), white },
        { "drawText transparent, no antialiasing    ", drawText(QImage::Format_ARGB32_Premultiplied, transparent, false), transparent },
        { "QLabel::render transparent               ", renderLabel(transparent), transparent },
        { "QLabel::render white                     ", renderLabel(white), white },
    };

    for (const auto& c : cases)
        std::cout << c.name << " visible pixels " << visiblePixels(c.img, c.bg) << std::endl;

    return 0;
}
