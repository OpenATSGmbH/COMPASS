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

#pragma once

#include <QLabel>
#include <QColor>
#include <QPoint>

/**
 * Label with a text shadow: the text is drawn a second time below it in the shadow color, shifted by the shadow
 * offset (e.g. readable text on a semi-transparent background, the shadow backs the antialiased glyph edges).
 * - plain text only: rich text, selectable text, labels with a buddy (mnemonics), pixmaps and movies are drawn by
 *   QLabel without shadow
 * - the shadow does not change the layout: size hints and text position as in QLabel (switching the shadow never
 *   resizes the label), a shadow beyond the widget rect is clipped (e.g. the last pixel column of a label at its size
 *   hint)
 * - with the shadow disabled the label behaves like a QLabel
 */
class StyledLabel : public QLabel
{
    Q_OBJECT

public:
    explicit StyledLabel(QWidget* parent = nullptr);
    explicit StyledLabel(const QString& text, QWidget* parent = nullptr);
    virtual ~StyledLabel() = default;

    void setShadowEnabled(bool enabled);
    void setShadowColor(const QColor& color);
    void setShadowOffset(const QPoint& offset);

    bool          isShadowEnabled() const { return shadow_enabled_; }
    const QColor& shadowColor() const { return shadow_color_; }
    const QPoint& shadowOffset() const { return shadow_offset_; }

    bool hasShadow() const;

    static const QColor ShadowColorDefault;
    static const QPoint ShadowOffsetDefault;

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    bool   shadow_enabled_ = true;
    QColor shadow_color_   = ShadowColorDefault;
    QPoint shadow_offset_  = ShadowOffsetDefault; // shadow position relative to the text in px
};
