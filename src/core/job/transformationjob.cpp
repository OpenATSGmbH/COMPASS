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

#include "TransformationJob.h"

#include "Logger.h"
#include "Transformation.h"
#include "WorkerThreadManager.h"

/**
 * Adds itself as "normal" job to WorkerThreadManager.
 *
 * \param transformation encapsulates working package
 */
TransformationJob::TransformationJob(JobOrderer* orderer, boost::function<void(Job*)> done_function,
                                     boost::function<void(Job*)> obsolete_function,
                                     Transformation* transformation)
    : Job(orderer, done_function, obsolete_function)
{
    WorkerThreadManager::getInstance().addJob(this);

    logdbg;

    traced_assert(transformation);
    transformation_ = transformation;
}

TransformationJob::~TransformationJob() {}

/**
 * Calls the transformation's execute function
 */
void TransformationJob::execute()
{
    logdbg;
    traced_assert(!done_);
    traced_assert(transformation_);
    transformation_->doExecute();
    done_ = true;
}

Transformation* TransformationJob::getTransformation()
{
    traced_assert(transformation_);
    return transformation_;
}
