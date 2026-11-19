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

#include "deckbutton.h"

#include <QApplication>
#include <QFontMetrics>
#include <QMouseEvent>

#include <algorithm>

const int    DeckButton::IconSize         = 50;
const int    DeckButton::LabelFontSize    = 9;
const int    DeckButton::LabelRows        = 2;
const double DeckButton::LabelWidthFactor = 1.1;

/**
 */
DeckButton::DeckButton(QWidget* parent)
:   QToolButton(parent)
{
    setFont(labelFont());
    setToolButtonStyle(Qt::ToolButtonStyle::ToolButtonTextUnderIcon);
    setIconSize(QSize(IconSize, IconSize));
    setCheckable(true);
}

/**
 */
QSize DeckButton::sizeForLabels(const std::vector<std::vector<std::string>>& labels, const QFont& font)
{
    const QFontMetrics fm(font);

    size_t max_rows = LabelRows;
    int    max_w    = 0;
    int    max_h    = fm.height();

    for (const auto& rows : labels)
    {
        max_rows = std::max(max_rows, rows.size());

        for (const auto& row : rows)
        {
            const auto r = fm.boundingRect(QString::fromStdString(row));

            max_w = std::max(max_w, r.width());
            max_h = std::max(max_h, r.height());
        }
    }

    const int text_w = max_w * LabelWidthFactor;
    const int text_h = max_rows * max_h * 1.3;

    return QSize(std::max(IconSize + 2, text_w), IconSize + 2 + text_h);
}

/**
 */
QString DeckButton::elidedLabel(const QString& label, const QSize& size, const QFont& font)
{
    const QFontMetrics fm(font);
    const int          max_w = size.width() / LabelWidthFactor;

    if (fm.boundingRect(label).width() <= max_w)
        return label;

    return fm.elidedText(label, Qt::ElideRight, max_w);
}

/**
 */
QFont DeckButton::labelFont()
{
    QFont font = QApplication::font("QToolButton");
    font.setPointSize(LabelFontSize);

    return font;
}

/**
 */
void DeckButton::mousePressEvent(QMouseEvent* e)
{
    if (e->button() == Qt::RightButton)
    {
        emit rightClickedSignal();
        return;
    }

    QToolButton::mousePressEvent(e);
}
