#include "catch.hpp"

#include "kalman_filter_um2d.h"

#include <cmath>
#include <random>

using namespace kalman;

// state layout of KalmanFilterUM2D: [ x, vx, y, vy ]

namespace
{

Matrix posMeasurementNoise(double sigma)
{
    Matrix R(2, 2);
    R.setZero();
    R(0, 0) = sigma * sigma;
    R(1, 1) = sigma * sigma;
    return R;
}

} // namespace

TEST_CASE("kalman um2d converges on constant-velocity motion", "[reconstruction][kalman]")
{
    const double vx      = 100.0; // m/s
    const double vy      =  50.0; // m/s
    const double dt      =   1.0; // s
    const double sigma   =  30.0; // m
    const double Q_var   =   4.0; // (2 m/s^2)^2 process noise
    const size_t n_steps =  60;

    std::mt19937 rng(1234);
    std::normal_distribution<double> noise(0.0, sigma);

    KalmanFilterUM2D filter_um2d(false); // measure positions only

    // note: KalmanFilterLinear declares protected predict/update/init
    // overloads that hide the public base-class API - call via base ref
    KalmanFilter& filter = filter_um2d;

    // init at first (noisy) position, unknown velocity
    Vector x0(4);
    x0 << noise(rng), 0.0, noise(rng), 0.0;

    Matrix P0(4, 4);
    P0.setZero();
    P0(0, 0) = sigma * sigma;
    P0(1, 1) = 100.0 * 100.0;
    P0(2, 2) = sigma * sigma;
    P0(3, 3) = 100.0 * 100.0;

    filter.init(x0, P0, dt, Q_var);

    Matrix R = posMeasurementNoise(sigma);

    for (size_t i = 1; i <= n_steps; ++i)
    {
        Vector z(2);
        z << vx * i * dt + noise(rng), vy * i * dt + noise(rng);

        REQUIRE(filter.predict(dt, Q_var) == KalmanError::NoError);
        REQUIRE(filter.update(z, R) == KalmanError::NoError);
    }

    const Vector& x = filter.getX();
    const Matrix& P = filter.getP();

    // state near ground truth
    CHECK(std::fabs(x(0) - vx * n_steps * dt) < 3.0 * sigma);
    CHECK(std::fabs(x(2) - vy * n_steps * dt) < 3.0 * sigma);

    // velocity estimated from positions alone
    CHECK(std::fabs(x(1) - vx) < 10.0);
    CHECK(std::fabs(x(3) - vy) < 10.0);

    // covariance symmetric, positive diagonal
    CHECK((P - P.transpose()).cwiseAbs().maxCoeff() < 1e-6);
    for (int i = 0; i < 4; ++i)
        CHECK(P(i, i) > 0.0);

    // filtered position uncertainty better than a single measurement
    CHECK(P(0, 0) < sigma * sigma);
    CHECK(P(2, 2) < sigma * sigma);
}

TEST_CASE("kalman um2d prediction grows uncertainty", "[reconstruction][kalman]")
{
    const double sigma = 30.0;
    const double Q_var = 4.0;

    KalmanFilterUM2D filter_um2d(false);
    KalmanFilter& filter = filter_um2d;

    Vector x0(4);
    x0 << 0.0, 100.0, 0.0, 0.0;

    Matrix P0(4, 4);
    P0.setZero();
    P0.diagonal() << sigma * sigma, 25.0, sigma * sigma, 25.0;

    filter.init(x0, P0, 1.0, Q_var);

    double last_var_x = filter.getP()(0, 0);

    for (int i = 0; i < 5; ++i)
    {
        REQUIRE(filter.predict(1.0, Q_var) == KalmanError::NoError);

        double var_x = filter.getP()(0, 0);
        CHECK(var_x > last_var_x);
        last_var_x = var_x;
    }

    // position moves along the velocity vector during prediction
    CHECK(filter.getX()(0) == Approx(500.0).margin(1e-6));
    CHECK(filter.getX()(2) == Approx(0.0).margin(1e-6));
}
