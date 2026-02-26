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

#include "buffer/buffer.h"
#include "rtcommand/rtcommand.h"
#include "reportdefs.h"

#include <vector>
#include <memory>

#include <boost/optional.hpp>

class COMPASS;
extern void init_evaluation_commands(COMPASS& compass);

/**
 * evaluate
 */
struct RTCommandEvaluate : public rtcommand::RTCommand
{
    bool        run_filter_ {false};
    std::string config_;
    std::string result_name_;

protected:
    virtual bool run_impl() override;

    DECLARE_RTCOMMAND(evaluate, "run evaluation")
    DECLARE_RTCOMMAND_OPTIONS
};
