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

#include "reconstructorbase.h"
#include "global.h"
#include "reconstructortarget.h"
#include "simpleassociator.h"
#include "referencecalculator.h"

class SimpleReconstructorSettings : public ReconstructorBaseSettings
{
  public:
    SimpleReconstructorSettings() {};

    double max_distance_notok_ {5*NM2M};
    double max_distance_dubious_ {2*NM2M};
    double max_distance_acceptable_ {1*NM2M};
};

class SimpleReconstructorWidget;

class SimpleReconstructor : public ReconstructorBase
{
    Q_OBJECT

  signals:
    void updateWidgetsSignal();

  public:
    SimpleReconstructor(const std::string& class_id, const std::string& instance_id,
                        ReconstructorTask& task, std::unique_ptr<AccuracyEstimatorBase>&& acc_estimator);
    virtual ~SimpleReconstructor();

    virtual SimpleAssociator& associator() override;
    virtual dbContent::VariableSet getReadSetFor(const std::string& dbcontent_name) const override;

    virtual void reset() override;

    virtual SimpleReconstructorSettings& settings() override;
    virtual const SimpleReconstructorSettings& settings() const override;

    SimpleReconstructorWidget* widget(); // ownage by caller

    virtual void updateWidgets() override;

    virtual void createAdditionalAnnotations() override;

    virtual const std::map<unsigned int, std::map<unsigned int,
                                                  std::pair<unsigned int, unsigned int>>>& assocAounts() const override
        { return associatior_.assocAounts(); };

  protected:
  
    friend class dbContent::ReconstructorTarget;
    friend class SimpleAssociator;
    friend class SimpleReferenceCalculator;

    SimpleReconstructorSettings settings_;
    SimpleAssociator associatior_;
    ReferenceCalculator ref_calculator_;

    virtual void processSlice_impl() override;
};

