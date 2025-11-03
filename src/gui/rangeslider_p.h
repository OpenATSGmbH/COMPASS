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

#include <QStyle>
#include <QObject>

#include "rangeslider.h"

QT_FORWARD_DECLARE_CLASS(QStylePainter)
QT_FORWARD_DECLARE_CLASS(QStyleOptionSlider)

/**
 */
class RangeSliderPrivate : public QObject, public qxt::QxtPrivate<RangeSlider>
{
    Q_OBJECT
public:
    QXT_DECLARE_PUBLIC(RangeSlider)

    RangeSliderPrivate();
    void initStyleOption(QStyleOptionSlider* option, RangeSlider::SpanHandle handle = RangeSlider::UpperHandle) const;
    int pick(const QPoint& pt) const
    {
        return qxt_p().orientation() == Qt::Horizontal ? pt.x() : pt.y();
    }
    int pixelPosToRangeValue(int pos) const;
    void handleMousePress(const QPoint& pos, QStyle::SubControl& control, int value, RangeSlider::SpanHandle handle);
    void drawHandle(QStylePainter* painter, RangeSlider::SpanHandle handle) const;
    void setupPainter(QPainter* painter, Qt::Orientation orientation, qreal x1, qreal y1, qreal x2, qreal y2) const;
    void drawSpan(QStylePainter* painter, const QRect& rect) const;
    void triggerAction(QAbstractSlider::SliderAction action, bool main);
    void swapControls();

    int lower;
    int upper;
    int lowerPos;
    int upperPos;
    int offset;
    int position;
    RangeSlider::SpanHandle lastPressed;
    RangeSlider::SpanHandle mainControl;
    QStyle::SubControl lowerPressed;
    QStyle::SubControl upperPressed;
    RangeSlider::HandleMovementMode movement;
    bool firstMovement;
    bool blockTracking;

public Q_SLOTS:
    void updateRange(int min, int max);
    void movePressedHandle();

private:
    RangeSlider* d_ptr;
};
