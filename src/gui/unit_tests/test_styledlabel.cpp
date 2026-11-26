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

#include "catch.hpp"
#include "styledlabel.h"

#include <QImage>
#include <QPalette>

namespace
{

const QColor TextColor   = QColor(255, 0, 0);
const QColor ShadowColor = QColor(0, 0, 255);

/**
 * Label content on a transparent image (no window background, as rendered into the label overlay).
 */
QImage render(QLabel& label)
{
    QPalette pal = label.palette();
    pal.setColor(QPalette::WindowText, TextColor);
    label.setPalette(pal);

    label.resize(label.sizeHint());

    QImage img(label.size(), QImage::Format_ARGB32_Premultiplied);
    img.fill(Qt::transparent);

    label.render(&img, QPoint(), QRegion(), QWidget::DrawChildren);

    return img.convertToFormat(QImage::Format_ARGB32);
}

/**
 * Number of pixels mainly of the shadow color (blue, hardly red).
 */
int shadowPixels(const QImage& img)
{
    int n = 0;
    for (int y = 0; y < img.height(); ++y)
    {
        for (int x = 0; x < img.width(); ++x)
        {
            const QRgb p = img.pixel(x, y);
            if (qAlpha(p) > 128 && qBlue(p) > 128 && qRed(p) < 64)
                ++n;
        }
    }
    return n;
}

}

TEST_CASE("StyledLabel without shadow renders like a QLabel", "[gui][styledlabel]")
{
    QLabel      plain("DLH4AB 3412 FL350");
    StyledLabel styled("DLH4AB 3412 FL350");
    styled.setShadowColor(ShadowColor);
    styled.setShadowEnabled(false);

    REQUIRE(!styled.hasShadow());
    REQUIRE(styled.sizeHint() == plain.sizeHint());
    REQUIRE(styled.minimumSizeHint() == plain.minimumSizeHint());

    REQUIRE(render(styled) == render(plain));
}

TEST_CASE("StyledLabel draws the shadow below the text", "[gui][styledlabel]")
{
    QLabel      plain("DLH4AB 3412 FL350");
    StyledLabel styled("DLH4AB 3412 FL350");
    styled.setShadowColor(ShadowColor);

    REQUIRE(styled.isShadowEnabled()); // default
    REQUIRE(styled.shadowOffset() == StyledLabel::ShadowOffsetDefault);
    REQUIRE(styled.hasShadow());

    SECTION("shadow pixels where the shifted text does not cover it, the layout stays as in QLabel")
    {
        const QImage img = render(styled);

        REQUIRE(shadowPixels(img) > 0);
        REQUIRE(shadowPixels(render(plain)) == 0);

        REQUIRE(styled.sizeHint() == plain.sizeHint());
        REQUIRE(styled.minimumSizeHint() == plain.minimumSizeHint());
    }

    SECTION("the text itself is drawn exactly like QLabel draws it")
    {
        //invisible shadow: only the text drawn by StyledLabel remains
        styled.setShadowColor(Qt::transparent);

        REQUIRE(render(styled) == render(plain));
    }

    SECTION("other offsets: shadow on the other side, layout unchanged")
    {
        styled.setShadowOffset(QPoint(-2, 3));

        REQUIRE(styled.sizeHint() == plain.sizeHint());
        REQUIRE(shadowPixels(render(styled)) > 0);
    }

    SECTION("switching the shadow does not change the size hints, disabled it renders like a QLabel")
    {
        const QSize size_hint = styled.sizeHint();

        styled.setShadowEnabled(false);

        REQUIRE(styled.sizeHint() == size_hint);
        REQUIRE(render(styled) == render(plain));
    }
}

TEST_CASE("StyledLabel draws no shadow for content QLabel draws differently", "[gui][styledlabel]")
{
    StyledLabel styled;
    styled.setShadowColor(ShadowColor);

    SECTION("rich text")
    {
        styled.setText("<b>DLH4AB</b>");
        REQUIRE(!styled.hasShadow());
        REQUIRE(shadowPixels(render(styled)) == 0);
    }

    SECTION("selectable text")
    {
        styled.setText("DLH4AB");
        styled.setTextInteractionFlags(Qt::TextSelectableByMouse);
        REQUIRE(!styled.hasShadow());
        REQUIRE(shadowPixels(render(styled)) == 0);
    }

    SECTION("empty text")
    {
        REQUIRE(!styled.hasShadow());
        REQUIRE(styled.sizeHint() == QLabel().sizeHint());
    }
}
