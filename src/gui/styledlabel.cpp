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

#include "styledlabel.h"

#include <QPainter>
#include <QStyle>
#include <QStyleOption>
#include <QTextDocument>

const QColor StyledLabel::ShadowColorDefault  = QColor(0, 0, 0);
const QPoint StyledLabel::ShadowOffsetDefault = QPoint(1, 1);

/**
 */
StyledLabel::StyledLabel(QWidget* parent)
:   QLabel(parent)
{
}

/**
 */
StyledLabel::StyledLabel(const QString& text, QWidget* parent)
:   QLabel(text, parent)
{
}

/**
 */
void StyledLabel::setShadowEnabled(bool enabled)
{
    if (shadow_enabled_ == enabled)
        return;

    shadow_enabled_ = enabled;

    update();
}

/**
 */
void StyledLabel::setShadowColor(const QColor& color)
{
    if (shadow_color_ == color)
        return;

    shadow_color_ = color;

    update();
}

/**
 */
void StyledLabel::setShadowOffset(const QPoint& offset)
{
    if (shadow_offset_ == offset)
        return;

    shadow_offset_ = offset;

    update();
}

/**
 * Checks if a shadow is drawn: enabled, with an offset, for plain text which QLabel draws as a single item
 * (rich and selectable text use a text document, labels with a buddy show mnemonics).
 */
bool StyledLabel::hasShadow() const
{
    if (!shadow_enabled_ || shadow_offset_.isNull() || text().isEmpty())
        return false;

    const bool rich_text = textFormat() == Qt::RichText ||
                           (textFormat() == Qt::AutoText && Qt::mightBeRichText(text()));
    const bool selectable = textInteractionFlags() & (Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);

    return !rich_text && !selectable && !buddy();
}

/**
 * Draws the plain text like QLabel (frame, margin, indent, alignment, word wrap, palette), first in the shadow color
 * shifted by the offset, then in the text color at the position QLabel uses. Without a shadow QLabel draws the label.
 */
void StyledLabel::paintEvent(QPaintEvent* event)
{
    if (!hasShadow())
    {
        QLabel::paintEvent(event);
        return;
    }

    QPainter painter(this);
    drawFrame(&painter);

    const Qt::LayoutDirection direction = text().isRightToLeft() ? Qt::RightToLeft : Qt::LeftToRight;
    const Qt::Alignment       align     = QStyle::visualAlignment(direction, alignment());

    //text rect as in QLabel: contents minus margin and indent
    QRect rect = contentsRect().adjusted(margin(), margin(), -margin(), -margin());

    int indent = this->indent();
    if (indent < 0 && frameWidth()) // no indent, but a frame
        indent = fontMetrics().horizontalAdvance(QLatin1Char('x')) / 2 - margin();

    if (indent > 0)
    {
        if (align & Qt::AlignLeft)
            rect.setLeft(rect.left() + indent);
        if (align & Qt::AlignRight)
            rect.setRight(rect.right() - indent);
        if (align & Qt::AlignTop)
            rect.setTop(rect.top() + indent);
        if (align & Qt::AlignBottom)
            rect.setBottom(rect.bottom() - indent);
    }

    int flags = align | (direction == Qt::LeftToRight ? Qt::TextForceLeftToRight : Qt::TextForceRightToLeft);
    if (wordWrap())
        flags |= Qt::TextWordWrap;

    QStyleOption opt;
    opt.initFrom(this);

    QPalette shadow_palette = opt.palette;
    shadow_palette.setColor(foregroundRole(), shadow_color_);

    style()->drawItemText(&painter, rect.translated(shadow_offset_), flags, shadow_palette, isEnabled(), text(), foregroundRole());
    style()->drawItemText(&painter, rect, flags, opt.palette, isEnabled(), text(), foregroundRole());
}
