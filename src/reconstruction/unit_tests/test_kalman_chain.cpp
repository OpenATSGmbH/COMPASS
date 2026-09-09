#include "catch.hpp"

#include "kalman_chain.h"
#include "test_reconstruction_common.h"

#include <algorithm>
#include <cmath>
#include <random>

using namespace reconstruction;
using namespace test_reconstruction;

namespace
{

std::unique_ptr<KalmanChain> makeChain(const std::vector<Measurement>& mms)
{
    auto chain = std::make_unique<KalmanChain>();
    chain->init(kalman::KalmanType::UMKalman2D);
    chain->setMeasurementGetFunc(
        [&mms](unsigned long mm_id) -> const Measurement& { return mms.at(mm_id); });
    return chain;
}

//the full ordering invariant, checked through the public API so the test does not depend
//on KalmanChain's internal validation helper
bool isOrdered(KalmanChain& chain)
{
    for (size_t i = 1; i < chain.size(); ++i)
        if (chain.getUpdate(i).t < chain.getUpdate(i - 1).t)
            return false;

    return true;
}

} // namespace

TEST_CASE("kalman chain adds chronologically and predicts", "[reconstruction][kalman_chain]")
{
    SyntheticTrack track;
    track.sigma_m = 30.0;

    const size_t n = 40;
    auto mms = track.generate(n);

    auto chain = makeChain(mms);

    for (size_t i = 0; i < n; ++i)
        REQUIRE(chain->add(i, mms[i].t, mms[i].t, true));

    REQUIRE(chain->size() == n);

    // timestamps ordered
    for (size_t i = 1; i < n; ++i)
        CHECK(chain->getUpdate(i).t > chain->getUpdate(i - 1).t);

    // prediction between updates lands near the true trajectory
    auto ts_mid = mms[20].t + boost::posix_time::milliseconds(500);
    REQUIRE(chain->canPredict(ts_mid));

    Measurement predicted;
    REQUIRE(chain->predict(&predicted, nullptr, nullptr, ts_mid));

    double lat_true, lon_true;
    {
        double t_s = (ts_mid - track.t0).total_milliseconds() / 1000.0;
        lat_true = track.base_lat + (track.vy_ms * t_s) / MetersPerDegLat;
        lon_true = track.base_lon + (track.vx_ms * t_s) / metersPerDegLon(track.base_lat);
    }

    CHECK(distanceM(lat_true, lon_true, predicted.lat, predicted.lon) < 5.0 * track.sigma_m);
}

TEST_CASE("kalman chain out-of-order inserts match chronological adds", "[reconstruction][kalman_chain]")
{
    SyntheticTrack track;
    track.sigma_m = 30.0;

    const size_t n = 30;
    auto mms = track.generate(n);

    // reference: chronological adds
    auto chain_ref = makeChain(mms);
    for (size_t i = 0; i < n; ++i)
        REQUIRE(chain_ref->add(i, mms[i].t, mms[i].t, true));

    // shuffled: even indices first, then odd indices inserted in between
    auto chain_ins = makeChain(mms);

    for (size_t i = 0; i < n; i += 2)
        REQUIRE(chain_ins->add(i, mms[i].t, mms[i].t, true));
    for (size_t i = 1; i < n; i += 2)
        REQUIRE(chain_ins->insert(i, mms[i].t, mms[i].t, true));

    while (chain_ins->needsReestimate())
        REQUIRE(chain_ins->reestimate());

    REQUIRE(chain_ins->size() == n);

    // both chains hold updates for the same timestamps in the same order
    for (size_t i = 0; i < n; ++i)
        REQUIRE(chain_ins->getUpdate(i).t == chain_ref->getUpdate(i).t);

    // states agree closely after reestimation
    for (size_t i = 0; i < n; ++i)
    {
        const auto& u_ref = chain_ref->getKalmanUpdate(i);
        const auto& u_ins = chain_ins->getKalmanUpdate(i);

        REQUIRE(u_ref.valid);
        REQUIRE(u_ins.valid);

        CHECK(distanceM(u_ref.lat, u_ref.lon, u_ins.lat, u_ins.lon) < 20.0);
    }
}

TEST_CASE("kalman chain rejects predictions too far from data", "[reconstruction][kalman_chain]")
{
    SyntheticTrack track;
    auto mms = track.generate(10);

    auto chain = makeChain(mms);
    for (size_t i = 0; i < mms.size(); ++i)
        REQUIRE(chain->add(i, mms[i].t, mms[i].t, true));

    // default prediction_max_tdiff_sec is 10 s
    CHECK(chain->canPredict(mms.back().t + boost::posix_time::seconds(5)));
    CHECK_FALSE(chain->canPredict(mms.back().t + boost::posix_time::seconds(60)));
}

/**
 * insertAt only validates the adjacencies the inserted element creates, instead of rescanning
 * the whole chain. That is exact only if the chain is ordered beforehand, so this drives a
 * pseudo random insertion order and verifies the global ordering afterwards with the full scan.
 */
TEST_CASE("kalman chain stays ordered under random insert order", "[reconstruction][kalman_chain]")
{
    SyntheticTrack track;
    track.sigma_m = 30.0;

    const size_t n = 120;
    auto mms = track.generate(n);

    auto chain = makeChain(mms);

    //deterministic shuffle, no dependency on a random device
    std::vector<size_t> order(n);
    for (size_t i = 0; i < n; ++i)
        order[ i ] = i;

    std::mt19937 rng (1234);
    std::shuffle(order.begin(), order.end(), rng);

    for (size_t cnt = 0; cnt < n; ++cnt)
    {
        size_t i = order[ cnt ];
        REQUIRE(chain->insert(i, mms[ i ].t, mms[ i ].t, false));

        //the full scan has to agree after every single insert
        REQUIRE(isOrdered(*chain));
    }

    REQUIRE(chain->size() == n);
    REQUIRE(isOrdered(*chain));

    //and the result is the chronological order
    for (size_t i = 1; i < n; ++i)
        REQUIRE(chain->getUpdate(i - 1).t <= chain->getUpdate(i).t);
}

/**
 * Appending in order must be accepted as well, that path reports a negative insertion index.
 */
TEST_CASE("kalman chain stays ordered when appended in order", "[reconstruction][kalman_chain]")
{
    SyntheticTrack track;
    track.sigma_m = 30.0;

    const size_t n = 50;
    auto mms = track.generate(n);

    auto chain = makeChain(mms);

    for (size_t i = 0; i < n; ++i)
    {
        REQUIRE(chain->insert(i, mms[ i ].t, mms[ i ].t, false));
        REQUIRE(isOrdered(*chain));
    }

    REQUIRE(chain->size() == n);
}
