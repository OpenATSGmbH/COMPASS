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

#include "compass_core_export.h"

#include <string>
#include <vector>

#include <QToolButton>

/**
 * Icon button with a label under the icon, used by the Flight Deck and the View Deck.
 * The check state shows the selection of the deck. The deck sets it, a click does not toggle it.
 */
class COMPASS_CORE_API DeckButton : public QToolButton
{
    Q_OBJECT
public:
    DeckButton(QWidget* parent = nullptr);
    virtual ~DeckButton() = default;

    /**
     * Common size of the buttons of a deck, from the labels of all its buttons (one entry per button,
     * one string per label row). Every button has room for LabelRows rows, so decks with shorter
     * labels get the same height.
     */
    static QSize sizeForLabels(const std::vector<std::vector<std::string>>& labels, const QFont& font);

    static const int IconSize;
    static const int LabelFontSize;
    static const int LabelRows;

signals:
    void rightClickedSignal();

protected:
    void mousePressEvent(QMouseEvent* e) override;
    void nextCheckState() override {}
};
