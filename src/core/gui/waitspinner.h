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

#include <QWidget>

class QPainter;
class QRectF;
class QTimer;

/**
 * Rotating wait icon (open arc in the disabled text color), e.g. for data being reloaded.
 */
namespace waitSpinner
{

const int Steps      = 12;  // icon positions per turn
const int IntervalMS = 100; // step interval in ms

void paint(QPainter* painter, const QRectF& rect, int step);

} // namespace waitSpinner

/**
 * Small widget showing the wait icon while spinning.
 * - keeps its size when not spinning (paints nothing), so a layout does not shift
 * - the timer only runs while spinning and the widget is visible
 */
class WaitSpinnerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit WaitSpinnerWidget(QWidget* parent = nullptr);
    virtual ~WaitSpinnerWidget() = default;

    void setSpinning(bool spinning);
    bool isSpinning() const { return spinning_; }

    QSize sizeHint() const override;

    static const int IconSize; // icon size in pixels

protected:
    void paintEvent(QPaintEvent* event) override;
    void showEvent(QShowEvent* event) override;
    void hideEvent(QHideEvent* event) override;

private:
    void tick();
    void updateTimer();

    QTimer* timer_    = nullptr;
    bool    spinning_ = false;
    int     step_     = 0;
};
