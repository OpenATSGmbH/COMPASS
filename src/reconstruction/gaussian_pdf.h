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

#include <Eigen/Dense>

#include <boost/optional.hpp>

#include <vector>

/**
 * https://stackoverflow.com/questions/41538095/evaluate-multivariate-normal-gaussian-density-in-c
*/
class GaussianPDF
{
public:
    enum class NormalizeMode
    {
        Sum = 0,
        Log
    };

    GaussianPDF(const Eigen::MatrixXd& P_cov);
    virtual ~GaussianPDF() = default;

    bool valid() const;

    boost::optional<double> likelihood(const Eigen::VectorXd& x, bool check_eps) const;
    boost::optional<double> logLikelihood(const Eigen::VectorXd& x, bool check_eps) const;

    static void normalizeLikelihoods(Eigen::VectorXd& likelihoods, 
                                     NormalizeMode mode, 
                                     bool debug = false);
    static void normalizeLikelihoods(std::vector<double>& likelihoods, 
                                     NormalizeMode mode, 
                                     bool debug = false);
    static double probabilityFromMahalanobisSqr(double d2_m, 
                                                unsigned int n);

    static const double LikelihoodEpsilon;
    static const double LogLikelihoodEpsilon;
    static const double NormalizeThreshold;
    static const double NormalizeSumMin;
    static const double ExpInputMin;
    static const double LogInputMin;
    static const double LogSqrt2Pi;

private:
    Eigen::LLT<Eigen::MatrixXd> chol_P_cov_;
};
