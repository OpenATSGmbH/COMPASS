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

#include "rtcommand/rtcommand.h"

class COMPASS;

extern void init_analyze_commands(COMPASS& compass);

/**
 * get_analyze_inspectors
 *
 * Lists the inspectors registered on the Analyze Data Source task for a
 * given DSType. Mirrors the introspection pattern of `get_eval_standards`:
 * the web settings page calls it to populate the per-DSType inspector
 * checkbox list.
 */
struct RTCommandGetAnalyzeInspectors : public rtcommand::RTCommand
{
public:
    RTCommandGetAnalyzeInspectors();

    std::string ds_type_;

protected:
    bool run_impl() override;
    bool checkResult_impl() override;

    DECLARE_RTCOMMAND(get_analyze_inspectors,
                      "list inspectors registered on the analyze data source task for a given DSType")
    DECLARE_RTCOMMAND_OPTIONS
};
