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

#include "colormapwidget.h"
#include "colorlegend.h"

#include "logger.h"

#include <QVBoxLayout>
#include <QGridLayout>
#include <QLabel>

/**
*/
ColorLegendWidget::ColorLegendWidget(QWidget* parent)
:   QWidget(parent)
{
    createUI();
}

/**
*/
ColorLegendWidget::~ColorLegendWidget() = default;

/**
*/
void ColorLegendWidget::setLegend(const ColorLegend& legend)
{
    legend_ = legend;

    updateUI();
}

/**
*/
void ColorLegendWidget::clear()
{
    legend_.clear();

    updateUI();
}

/**
*/
void ColorLegendWidget::createUI()
{
    layout_ = new QHBoxLayout;
    setLayout(layout_);
}

/**
*/
void ColorLegendWidget::updateUI()
{
    if (widget_)
    {
        delete widget_;
        widget_ = nullptr;
    }

    if (legend_.empty())
        return;

    widget_ = new QWidget;

    QVBoxLayout* layout = new QVBoxLayout;
    layout->setContentsMargins(0, 0, 0, 0);
    widget_->setLayout(layout);

    QGridLayout* layout_grid = new QGridLayout;
    layout_grid->setContentsMargins(0, 0, 0, 0);
    layout->addLayout(layout_grid);
    layout->addStretch(1);

    int row = 0;
    for (const auto& entry : legend_.entries())
    {
        QImage img(16, 16, QImage::Format_ARGB32);
        img.fill(entry.first);

        QLabel* l = new QLabel;
        l->setPixmap(QPixmap::fromImage(img));
        l->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);

        QLabel* l2 = new QLabel;
        l2->setText(QString::fromStdString(entry.second));
        l2->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

        layout_grid->addWidget(l , row, 0);
        layout_grid->addWidget(l2, row, 1);

        ++row;
    }

    layout_->addWidget(widget_);
}
