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

#include "kalman_projection.h"
#include "kalman_interface.h"

#include "projection/projector.h"

#include "logger.h"

namespace reconstruction
{

/**
*/
KalmanProjectionHandler::KalmanProjectionHandler()
:   proj_(new FrameProjector)
{
    traced_assert(proj_);
}

/**
*/
KalmanProjectionHandler::~KalmanProjectionHandler() = default;

/**
 * Check if a valid projection is set.
 */
bool KalmanProjectionHandler::valid() const
{
    return (proj_ && proj_->valid());
}

/**
 * Initializes the map projection depending on the current projection mode.
 */
void KalmanProjectionHandler::initProjection(double lat, 
                                             double lon)
{
    //update to measurement location
    proj_->update(lat, lon);
    traced_assert(proj_->valid());

    //loginf << "start" << proj_->centerLat() << "," << proj_->centerLon() << " (" << proj_->centerCart().x() << "," << proj_->centerCart().y() << ")";
}

/**
 * Projects the given geodetic coords using the current map projection.
 */
void KalmanProjectionHandler::project(double& x, 
                                      double& y, 
                                      double lat, 
                                      double lon) const
{
    //project using current projection
    proj_->project(x, y, lat, lon);
}

/**
 * Unprojects the given cartesian coordinate to geodetic using the given projection center.
 * Note: The projection center is cached in the projector, so calling this function
 * with the same center multiple times will not result in a map projection creation overhead.
 */
void KalmanProjectionHandler::unproject(double& lat, 
                                        double& lon, 
                                        double x, 
                                        double y, 
                                        const Eigen::Vector2d* proj_center) const
{
    if (proj_center)
    {
        proj_->update(proj_center->x(), proj_center->y());
        traced_assert(proj_->valid());
    }
    proj_->unproject(lat, lon, x, y);
}

/**
 * Unprojects the given references using their respective map projection centers.
*/
// void KalmanProjectionHandler::unproject(std::vector<Reference>& references,
//                                         const std::vector<Eigen::Vector2d>& proj_centers) const
// {
//     //nothing to do?
//     if (settings_.map_proj_mode == MapProjectionMode::MapProjectNone)
//         return;

//     traced_assert(references.size() == proj_centers.size());

//     for (size_t i = 0; i < references.size(); ++i)
//         unproject(references[ i ].lat, references[ i ].lon, references[ i ].x, references[ i ].y, proj_centers[ i ]);
// }

/**
*/
bool KalmanProjectionHandler::inRangeCart(double x_cart, double y_cart) const
{
    //const auto& center = proj_->centerCart();

    //const double d_sqr = (Eigen::Vector2d(center.x(), center.y()) - Eigen::Vector2d(x_cart, y_cart)).squaredNorm();

    const double d_sqr = Eigen::Vector2d(x_cart, y_cart).squaredNorm();
    if (d_sqr > settings_.proj_max_dist_cart_sqr)
        return false; 

    return true;
}

/**
*/
bool KalmanProjectionHandler::inRangeWGS84(double lat, double lon) const
{
    if (std::fabs(proj_->centerLat() - lat) > settings_.proj_max_dist_wgs84 ||
        std::fabs(proj_->centerLon() - lon) > settings_.proj_max_dist_wgs84)
        return false;

    return true;
}

/**
 * Checks if the currently set map projection is still valid or outdated.
 */
bool KalmanProjectionHandler::needsReprojectionChange(kalman::KalmanUpdate& update,
                                                      const KalmanInterface& interface) const
{
    if (settings_.proj_dist_check == MapProjDistanceCheck::WGS84)
    {
        //check wgs84 distance
        if (!inRangeWGS84(update.lat, update.lon))
            return true;
    }
    else if (settings_.proj_dist_check == MapProjDistanceCheck::Cart)
    {
        //obtain cartesian coordinates of state
        double x_cart, y_cart;
        interface.xPos(x_cart, y_cart, update.state.x);

        if (!inRangeCart(x_cart, y_cart))
            return true;
    }

    return false;
}

/**
 * Changes the current map projection to the given kalman state's wgs84 coordinates,
 * and updates the kalman state accordingly.
 */
void KalmanProjectionHandler::changeProjection(kalman::KalmanUpdate& update,
                                               const KalmanInterface& interface)
{
    //handle imm state if available
    if (update.state.imm_state)
    {
        size_t n = update.state.imm_state->filter_states.size();

        //obtain lat lon for each submodel state
        double x_cart, y_cart, lat, lon;
        for (size_t i = 0; i < n; ++i)
        {
            //unproject sub-model position to lat-lon
            interface.xPos(x_cart, y_cart, update.state.imm_state->filter_states[ i ].x, (int)i);
            unproject(lat, lon, x_cart, y_cart);

            //temporarily store lat-lon to submodel state
            interface.xPos(update.state.imm_state->filter_states[ i ].x, lat, lon, (int)i);
        }
    }

    //obtain cartesian coordinates of state
    double x_cart, y_cart;
    interface.xPos(x_cart, y_cart, update.state.x);

    //obtain geodetic coordinates using current projection
    double lat, lon;
    unproject(lat, lon, x_cart, y_cart);

    //update projection to new geodetic coords
    initProjection(lat, lon);

    //update cartesian coords of state to new map projection
    //const auto& center_cart = proj_->centerCart();

    //store new position to state vector
    //note: we assume only small changes in map projection, 
    //so directions, velocities, accelerations and connected uncertainties 
    //are held constant during a projection change.
    interface.xPos(update.state.x, 0, 0);
    
    //handle imm state if available
    if (update.state.imm_state)
    {
        size_t n = update.state.imm_state->filter_states.size();

        double lat, lon, x_cart, y_cart;
        for (size_t i = 0; i < n; ++i)
        {
            //get temporarily stored lat-lon
            interface.xPos(lat, lon, update.state.imm_state->filter_states[ i ].x, (int)i);

            //project to new coord sys
            project(x_cart, y_cart, lat, lon);

            //store to submodel-state
            interface.xPos(update.state.imm_state->filter_states[ i ].x, x_cart, y_cart, (int)i);
        }
    }
}

/**
 * Checks if a projection change is needed, and if yes changes it and returns true.
 * Returns false otherwise.
 */
bool KalmanProjectionHandler::changeProjectionIfNeeded(kalman::KalmanUpdate& update,
                                                       const KalmanInterface& interface)
{
    if (!needsReprojectionChange(update, interface))
        return false;

    changeProjection(update, interface);

    return true;
}

/**
 * Reprojects the given state vector to a new map projection.
 */
void KalmanProjectionHandler::xReprojected(kalman::Vector& x_repr,
                                           const KalmanInterface& interface,
                                           const kalman::Vector& x,
                                           const Eigen::Vector2d& proj_center,
                                           const Eigen::Vector2d& proj_center_new,
                                           size_t* num_proj_center_changed)
{
    traced_assert(proj_);

    x_repr = x;

    if(num_proj_center_changed)
        *num_proj_center_changed = 0;

    //projection centers do match => nothing more to do
    if (proj_center == proj_center_new)
        return;

    //set old projection
    
    //obtain cartesian position from state vector
    double x_pos_cart, y_pos_cart;
    interface.xPos(x_pos_cart, y_pos_cart, x);

    bool changed0 = proj_center != projectionCenter();

    //unproject to geodetic using old projection center
    double pos_lat, pos_lon;
    unproject(pos_lat, pos_lon, x_pos_cart, y_pos_cart, &proj_center);

    bool changed1 = proj_center != proj_center_new;

    //project to cartesian using new projection center
    proj_->update(proj_center_new.x(), proj_center_new.y());
    proj_->project(x_pos_cart, y_pos_cart, pos_lat, pos_lon);

    //set new position in state vector
    interface.xPos(x_repr, x_pos_cart, y_pos_cart);

    if (num_proj_center_changed)
        *num_proj_center_changed = changed0 ? (changed1 ? 2 : 1) : (changed1 ? 1 : 0);
}

/**
 * Generates a state vector transformation from one map projection system into another one.
 * Used e.g. for rts smoothing or kalman state interpolation.
 */
kalman::XTransferFunc KalmanProjectionHandler::reprojectionTransform(const std::vector<kalman::KalmanUpdate>* updates,
                                                                     const KalmanInterface* interface,
                                                                     size_t offset)
{
    auto repr_trafo = [ = ] (kalman::Vector& x_tr, const kalman::Vector& x, size_t idx_old, size_t idx_new)
    {
        //data for the map reprojection is provided by the indices into the given kalman updates
        this->xReprojected(x_tr, *interface, x, (*updates)[offset + idx_old].projection_center, (*updates)[offset + idx_new].projection_center);
    };
    return repr_trafo;
}

/**
 * Returns the current projection center.
 * Note: just returns (0,0) if no valid projection is set.
*/
Eigen::Vector2d KalmanProjectionHandler::projectionCenter() const
{
    return (proj_->valid() ? Eigen::Vector2d(proj_->centerLat(), proj_->centerLon()) : Eigen::Vector2d(0, 0));
}

} // namespace reconstruction
