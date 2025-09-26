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

#include "traced_assert.h"

#include <Eigen/Core>

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace kalman
{

enum KalmanType
{
    UMKalman2D     = 0, // uniform motion kalman in the plane
    AMKalman2D,         // accelerated motion kalman in the plane
    IMMKalman2D,        // mixed kalman model filter in the plane
    UMKalman2DFull      // uniform motion kalman in the plane with full state
};

enum SmoothFailStrategy
{
    None = 0,
    Stop,
    SetInvalid
};

enum class KalmanError
{
    NoError = 0,
    Numeric,
    InvalidState,
    Unknown
};

enum KalmanInfoFlags
{
    InfoState        = 1 << 0,
    InfoStateExt     = 1 << 1,
    InfoIntermSteps  = 1 << 2,
    InfoAll          = 255
};

typedef Eigen::MatrixXd         Matrix;
typedef Eigen::VectorXd         Vector;
typedef boost::optional<Matrix> OMatrix;
typedef boost::optional<Vector> OVector;

/**
 * Transfer func for a state vector.
 * Used to transfer the given state vector between two collected kalman states.
 * This functional is kept very general on purpose and can be used e.g. to convert from
 * one map projection system to another one.
 * params: state vec transformed | state vec | index of the old kalman state | index of the new kalman state
 */
typedef std::function<void(Vector&,const Vector&,size_t,size_t)> XTransferFunc;

/**
 */
struct ProbState
{
    ProbState() {}
    ProbState(const Vector& _x, const Matrix& _P) : x(_x), P(_P) {}
    ProbState(const ProbState& other)
    :   x(other.x)
    ,   P(other.P) {}
    ProbState(ProbState&& other)
    :   x(other.x)
    ,   P(other.P) {}

    ProbState& operator=(const ProbState& other)
    {
        x = other.x;
        P = other.P;

        return *this;
    }

    ProbState& operator=(ProbState&& other)
    {
        x = other.x;
        P = other.P;

        return *this;
    }

    bool operator==(const ProbState& other) const
    {
        if (x  != other.x ||
            P  != other.P)
            return false;

        return true;
    }

    bool operator!=(const ProbState& other) const
    {
        return !operator==(other);
    }

    virtual std::string print() const
    {
        std::stringstream ss;
        ss << "x:\n" << x << "\n"
           << "P:\n" << P << "\n";

        return ss.str();
    }

    Vector x; // state
    Matrix P; // state uncertainty
};

/**
 */
struct GeoProbState : public ProbState
{
    GeoProbState() {}
    GeoProbState(const Vector& _x, const Matrix& _P, const Eigen::Vector2d& _proj_center) : ProbState(x, P), proj_center(_proj_center) {}
    GeoProbState(const GeoProbState& other)
    :   ProbState  (other)
    ,   proj_center(other.proj_center) {}
    GeoProbState(GeoProbState&& other)
    :   ProbState  (other)
    ,   proj_center(other.proj_center) {}

    GeoProbState& operator=(const GeoProbState& other)
    {
        ProbState::operator=(other);
        proj_center = other.proj_center;

        return *this;
    }

    GeoProbState& operator=(GeoProbState&& other)
    {
        ProbState::operator=(other);
        proj_center = other.proj_center;

        return *this;
    }

    bool operator==(const GeoProbState& other) const
    {
        if (!ProbState::operator==(other))
            return false;

        if (proj_center != other.proj_center)
            return false;

        return true;
    }

    bool operator!=(const GeoProbState& other) const
    {
        return !operator==(other);
    }

    virtual std::string print() const override
    {
        std::stringstream ss;
        ss << ProbState::print();

        ss << "proj_center:\n" << proj_center << "\n";

        return ss.str();
    }

    Eigen::Vector2d proj_center;
};

/**
*/
struct BasicKalmanState : public ProbState
{
    BasicKalmanState() {}
    BasicKalmanState(const Vector& _x, const Matrix& _P) : ProbState(_x, _P) {}
    BasicKalmanState(const BasicKalmanState& other)
    :   ProbState(other      )
    ,   F        (other.F    )
    ,   Q        (other.Q    )
    ,   debug    (other.debug) {}
    BasicKalmanState(BasicKalmanState&& other)
    :   ProbState(other      )
    ,   F        (other.F    )
    ,   Q        (other.Q    )
    ,   debug    (other.debug) {}

    BasicKalmanState& operator=(const BasicKalmanState& other)
    {
        ProbState::operator=(other);

        F     = other.F;
        Q     = other.Q;
        debug = other.debug;

        return *this;
    }

    BasicKalmanState& operator=(BasicKalmanState&& other)
    {
        ProbState::operator=(other);

        F     = other.F;
        Q     = other.Q;
        debug = other.debug;

        return *this;
    }

    bool operator==(const BasicKalmanState& other) const
    {
        if (!ProbState::operator==(other))
            return false;
        
        if (F != other.F ||
            Q != other.Q)
            return false;

        return true;
    }

    bool operator!=(const BasicKalmanState& other) const
    {
        return !operator==(other);
    }

    virtual std::string print() const override
    {
        std::stringstream ss;
        ss << ProbState::print();

        ss << "F:\n" << F << "\n"
           << "Q:\n" << Q << "\n";

        return ss.str();
    }

    virtual void debugState() { debug = true; }

    Matrix F;             // transition mat
    Matrix Q;             // process noise mat
    bool   debug = false; // debug this state
};

/**
*/
struct KalmanState : public BasicKalmanState
{
    /**
     * State for IMM filter
    */
    struct IMMState
    {
        // individual sub-filter states
        std::vector<BasicKalmanState> filter_states; 

        // current IMM state
        Vector mu;     // sub-filter probabilities
        Matrix omega;  // transition probabilities
        Vector cbar;   // normailization factors 
    };

    KalmanState() {}
    KalmanState(const Vector& _x, const Matrix& _P) : BasicKalmanState(_x, _P) {}
    KalmanState(const Vector& _x, const Matrix& _P, double _dt) : BasicKalmanState(_x, _P), dt(_dt) {}
    KalmanState(const Vector& _x, const Matrix& _P, double _dt, double _Q_var) : BasicKalmanState(_x, _P), dt(_dt), Q_var(_Q_var) {}
    KalmanState(const BasicKalmanState& basic_state) : BasicKalmanState(basic_state) {}

    KalmanState(const KalmanState& other)
    :   BasicKalmanState(other)
    ,   dt   (other.dt   )
    ,   Q_var(other.Q_var)
    {
        if (other.imm_state)
        {
            imm_state.reset(new IMMState);
            *imm_state = *other.imm_state;
        }
    }
    KalmanState(KalmanState&& other)
    :   BasicKalmanState(other)
    ,   dt   (other.dt   )
    ,   Q_var(other.Q_var)
    {
        if (other.imm_state)
            imm_state = std::move(other.imm_state);
    }

    KalmanState& operator=(const KalmanState& other)
    {
        BasicKalmanState::operator=(other);

        dt    = other.dt;
        Q_var = other.Q_var;

        if (other.imm_state)
            immState() = *other.imm_state;
        else if (imm_state)
            imm_state.reset();

        return *this;
    }

    KalmanState& operator=(KalmanState&& other)
    {
        BasicKalmanState::operator=(other);

        dt    = other.dt;
        Q_var = other.Q_var;

        if (other.imm_state)
            imm_state = std::move(other.imm_state);
        else if (imm_state)
            imm_state.reset();

        return *this;
    }

    bool operator==(const KalmanState& other) const
    {
        if (!BasicKalmanState::operator==(other))
            return false;

        if (dt    != other.dt || 
            Q_var != other.Q_var)
            return false;

        bool has_imm       = imm_state != nullptr;
        bool has_imm_other = other.imm_state != nullptr;

        if (has_imm != has_imm_other)
            return false;

        if (has_imm)
        {
            if (imm_state->mu != other.imm_state->mu ||
                imm_state->cbar != other.imm_state->cbar ||
                imm_state->omega != other.imm_state->omega)
                return false;

            if (imm_state->filter_states.size() != other.imm_state->filter_states.size())
                return false;

            size_t n = imm_state->filter_states.size();

            for (size_t i = 0; i < n; ++i)
                if (imm_state->filter_states[ i ] != other.imm_state->filter_states[ i ])
                    return false;
        }

        return true;
    }

    bool operator!=(const KalmanState& other) const
    {    
        return !operator==(other);
    }

    std::string print() const override final
    {
        std::stringstream ss;
        ss << "dt: " << dt << "\n"
           << "Q_var: " << Q_var << "\n"
           << BasicKalmanState::print();

        if (imm_state)
        {
            ss << "mu:\n" << imm_state->mu << "\n"
               << "omega:\n" << imm_state->omega << "\n"
               << "cbar:\n" << imm_state->cbar << "\n\n";

            for (size_t i = 0; i < imm_state->filter_states.size(); ++i)
                ss << "[Filter" << i << "]\n"
                   << imm_state->filter_states[ i ].print() << "\n";
        }

        return ss.str();
    }

    void debugState() override final
    {
        BasicKalmanState::debugState();

        if (imm_state)
        {
            for (auto& fs : imm_state->filter_states)
                fs.debugState();
        }
    }

    IMMState& immState()
    {
        if (!imm_state)
            imm_state.reset(new IMMState);
        return *imm_state;
    }

    std::unique_ptr<IMMState> imm_state; // optional IMM filter state

    double dt    = 0.0; // used timestep
    double Q_var = 0.0; // used process noise variance
};

/**
 * Minimal information about a kalman update's state, coord system and validity.
 */
struct KalmanUpdateMinimal
{
    void resetFlags()
    {
        valid = false;
    }

    std::string print()
    {
        std::stringstream ss;
        ss << "x: \n" << x << "\n"
           << "P: \n" << P << "\n"
           //<< "t: " << Utils::Time::toString(t) << "\n"
           << "projection center: " << projection_center.x() << " " << projection_center.y() << "\n"
           << "lat: " << lat << "\n"
           << "lon: " << lon << "\n"
           << "Q_var: " << Q_var << "\n"
           << "has wgs84: " << has_wgs84_pos << "\n"
           << "valid: " << valid;

        return ss.str();
    }

    Vector                   x;                 // state vector
    Matrix                   P;                 // covariance matrix
    boost::posix_time::ptime t;                 // time of update
    Eigen::Vector2d          projection_center; // center of the local stereographic projection used for this update
    double                   lat;               // latitude of state position (do not confuse with center of local projection!)
    double                   lon;               // longitude of state position (do not confuse with center of local projection!)
    double                   Q_var;             // used process noise variance

    bool has_wgs84_pos = false; // lat/long are set
    bool valid         = false; // kalman update is valid
};

//unique update id which also accounts for interpolated source data
typedef std::pair<unsigned long, boost::posix_time::ptime> UniqueUpdateID;

/**
 * Low-level struct for a kalman update.
 * Contains everything needed for postprocessing and extraction of a final reference.
 */
struct KalmanUpdate
{
    KalmanUpdate() {}
    KalmanUpdate(const KalmanUpdateMinimal& update_min)
    {
        state.x     = update_min.x;
        state.P     = update_min.P;
        state.F     = kalman::Matrix();
        state.Q     = kalman::Matrix();
        state.dt    = 0.0;   // not defined by min update
        state.Q_var = update_min.Q_var;
        state.debug = false; // not defined by min update

        projection_center = update_min.projection_center;
        lat               = update_min.lat;
        lon               = update_min.lon;
        t                 = update_min.t;
        has_wgs84_pos     = update_min.has_wgs84_pos;
        valid             = update_min.valid;
        reinit            = false; // not defined by min update
        proj_changed      = false; // not defined by min update
        
    }

    void resetFlags()
    {
        has_wgs84_pos = false;
        valid         = false;
        reinit        = false;
        proj_changed  = false;
    }

    void minimalInfo(KalmanUpdateMinimal& info) const
    {
        info.x                 = state.x;
        info.P                 = state.P;
        info.t                 = t;
        info.projection_center = projection_center;
        info.lat               = lat;
        info.lon               = lon;
        info.Q_var             = state.Q_var;
        info.has_wgs84_pos     = has_wgs84_pos;
        info.valid             = valid;
    };

    KalmanUpdateMinimal minimalInfo() const
    {
        KalmanUpdateMinimal info;
        minimalInfo(info);

        return info;
    }

    void debugUpdate() 
    { 
        state.debugState(); 
    }

    bool isDebug() const
    {
        return state.debug;
    }

    UniqueUpdateID uniqueID() const
    {
        traced_assert(source_id.has_value());
        return UniqueUpdateID(source_id.value(), t);
    }

    KalmanState              state;             // kalman internal state, can be used for rts smooting, state interpolation, etc.
    Eigen::Vector2d          projection_center; // center of the local stereographic projection used for this update
    double                   lat{0};               // latitude of state position (do not confuse with center of local projection!)
    double                   lon{0};               // longitude of state position (do not confuse with center of local projection!)
    boost::posix_time::ptime t;                 // time of update

    boost::optional<unsigned long> source_id;    // optional source id
    boost::optional<float>         Q_var_interp; // optional interpolation process noise variance

    bool has_wgs84_pos = false; // lat/long are set
    bool valid         = false; // kalman update is valid
    bool reinit        = false; // kalman was reinitialized at this update, represents the beginning of a new kalman chain
    bool proj_changed  = false; // the projection center changed during this update
};

/**
*/
struct RTSStepInfo
{
    BasicKalmanState state0;
    BasicKalmanState state1;
    BasicKalmanState state0_smooth;
    BasicKalmanState state1_smooth;
};

/**
*/
struct RTSDebugInfo
{
    void addStepInfo(const RTSStepInfo& step_info)
    {
        state0.immState().filter_states.push_back(step_info.state0);
        state1.immState().filter_states.push_back(step_info.state1);
        state0_smooth.immState().filter_states.push_back(step_info.state0_smooth);
        state1_smooth.immState().filter_states.push_back(step_info.state1_smooth);
    }

    KalmanState     state0;
    KalmanState     state1;
    KalmanState     state0_smooth;
    KalmanState     state1_smooth;
    size_t          update_idx;
    Eigen::Vector2d projection_center;
    Vector          mu;
};

} // namespace kalman
