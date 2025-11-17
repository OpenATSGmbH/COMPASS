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

#include "evaluation_commands.h"
#include "rtcommand/rtcommand_macros.h"
#include "rtcommand_registry.h"
#include "compass.h"
#include "evaluationmanager.h"
#include "evaluationmanagerwidget.h"
#include "mainwindow.h"
#include "compass.h"
#include "dbcontentmanager.h"

#include <boost/program_options.hpp>

REGISTER_RTCOMMAND(RTCommandEvaluate)

/**
*/
void init_evaluation_commands()
{
    RTCommandEvaluate::init();
}

/***************************************************************************************
 * RTCommandEvaluate
 ***************************************************************************************/

bool RTCommandEvaluate::run_impl()
{
    if (!COMPASS::instance().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (COMPASS::instance().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+COMPASS::instance().appModeStr());
        return false;
    }

    if (run_filter_)
        COMPASS::instance().dbContentManager().autoFilterUTNS();

    EvaluationManager& eval_man = COMPASS::instance().evaluationManager();

    auto can_evaluate = eval_man.canEvaluate();
    if (!can_evaluate.ok())
    {
        setResultMessage("Unable to load evaluation data and evaluate: " + can_evaluate.error());
        return false;
    }

    loginf << "loading evaluation data";

    auto res = eval_man.evaluate(false);
    if (!res.ok())
    {
        setResultMessage(res.error());
        return false;
    }

    return true;
}

void RTCommandEvaluate::collectOptions_impl(OptionsDescription& options,
                                          PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("run_filter,f", "run evaluation filter before evaluation");
}

void RTCommandEvaluate::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_CHECK_VAR(variables, "run_filter", run_filter_)
}
