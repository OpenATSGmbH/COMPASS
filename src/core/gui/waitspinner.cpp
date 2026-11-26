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

#include "waitspinner.h"

#include <algorithm>

#include <QApplication>
#include <QPainter>
#include <QPen>
#include <QTimer>

namespace waitSpinner
{

/**
 * Open arc in the rect, rotated clockwise by the given step.
 */
void paint(QPainter* painter, const QRectF& rect, int step)
{
    QPen pen(QApplication::palette().color(QPalette::Disabled, QPalette::Text), 2.0);
    pen.setCapStyle(Qt::RoundCap);

    painter->save();
    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->setPen(pen);

    // Qt angles are counter-clockwise in 1/16 degrees
    const int start = -step * (360 / Steps) * 16;
    painter->drawArc(rect.adjusted(1, 1, -1, -1), start, 270 * 16);

    painter->restore();
}

} // namespace waitSpinner

/***************************************************************************
 * WaitSpinnerWidget
 ***************************************************************************/

const int WaitSpinnerWidget::IconSize = 12;

/**
 */
WaitSpinnerWidget::WaitSpinnerWidget(QWidget* parent)
:   QWidget(parent)
{
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    timer_ = new QTimer(this);
    timer_->setInterval(waitSpinner::IntervalMS);
    connect(timer_, &QTimer::timeout, this, &WaitSpinnerWidget::tick);
}

/**
 */
void WaitSpinnerWidget::setSpinning(bool spinning)
{
    if (spinning_ == spinning)
        return;

    spinning_ = spinning;

    updateTimer();
    update();
}

/**
 */
QSize WaitSpinnerWidget::sizeHint() const
{
    return QSize(IconSize + 4, IconSize + 4);
}

/**
 */
void WaitSpinnerWidget::paintEvent(QPaintEvent* event)
{
    (void)event;

    if (!spinning_)
        return;

    const int    size = std::min(std::min(width(), height()), IconSize);
    const QRectF rect((width() - size) / 2.0, (height() - size) / 2.0, size, size);

    QPainter painter(this);
    waitSpinner::paint(&painter, rect, step_);
}

/**
 */
void WaitSpinnerWidget::showEvent(QShowEvent* event)
{
    QWidget::showEvent(event);
    updateTimer();
}

/**
 */
void WaitSpinnerWidget::hideEvent(QHideEvent* event)
{
    QWidget::hideEvent(event);
    updateTimer();
}

/**
 */
void WaitSpinnerWidget::tick()
{
    step_ = (step_ + 1) % waitSpinner::Steps;
    update();
}

/**
 */
void WaitSpinnerWidget::updateTimer()
{
    const bool run = spinning_ && isVisible();

    if (run && !timer_->isActive())
        timer_->start();
    else if (!run && timer_->isActive())
        timer_->stop();
}
