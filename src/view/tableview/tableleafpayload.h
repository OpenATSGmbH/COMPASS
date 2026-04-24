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

#include "dbcontentlayer.h"

#include <QColor>

#include <string>

/**
 * DBContentLeafPayload adapter for Table View.
 *
 * Unlike scatter, table view has no per-layer data object — layers are
 * derived by scanning buffers at rebuild time. A payload holds just the
 * layer identity, its row count, and its chart color. Visibility flips are
 * tracked on the payload itself; the data widget reacts to the model's
 * hiddenChangedSignal once per user action to refilter + redraw.
 *
 * Persisted id matches scatter's key convention
 * ("<ds_type>:<ds_name>:L<n>:<dbcontent>") so hidden state round-trips
 * across reloads and view-point restores.
 */
class TableLeafPayload : public DBContentLeafPayload
{
public:
    TableLeafPayload(const std::string& full_key,
                     unsigned int count,
                     const QColor& color)
        : full_key_(full_key)
        , count_(count)
        , color_(color)
    {}

    std::string name() const override { return full_key_; }
    unsigned int count() const override { return count_; }
    QColor color() const override { return color_; }

    void setVisible(bool v) override { visible_ = v; }

    std::string persistenceId() const override { return full_key_; }

    bool visible() const { return visible_; }

private:
    std::string  full_key_;
    unsigned int count_ {0};
    QColor       color_;
    bool         visible_ {true};
};
