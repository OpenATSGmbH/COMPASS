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

#include "configurable.h"
#include "singleton.h"
#include "buffer.h"

#include "gdal_priv.h"

// fix stupid gdal constant
#ifdef PACKAGE_VERSION
#undef PACKAGE_VERSION
#endif

#include <GeographicLib/MagneticModel.hpp>

#include <memory>

class ProjectionManagerWidget;
class Projection;
class GeoProjection;

class ProjectionManager : public Singleton, public Configurable
{
protected:
    ProjectionManager();

public:

    virtual ~ProjectionManager();

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id);

    ProjectionManagerWidget* widget();
    void deleteWidget();

    static ProjectionManager& instance()
    {
        static ProjectionManager instance;
        return instance;
    }
    std::string currentProjectionName() const;
    void currentProjectionName(const std::string& name);

    bool hasProjection(const std::string& name);
    Projection& projection(const std::string& name);
    bool hasCurrentProjection();
    Projection& currentProjection();

    std::map<std::string, std::unique_ptr<Projection>>& projections();

    // in place calculation, returns transformation errors count
    unsigned int doRadarPlotPositionCalculations (std::map<std::string, std::shared_ptr<Buffer>> buffers);
    unsigned int doXYPositionCalculations (std::map<std::string, std::shared_ptr<Buffer>> buffers);
    // returns transformation errors count, update buffers
    std::pair<unsigned int, std::map<std::string, std::shared_ptr<Buffer>>>
      doUpdateRadarPlotPositionCalculations (std::map<std::string, std::shared_ptr<Buffer>> buffers);

    static const std::string RS2G_NAME;
    //static const std::string OGR_NAME;
    //static const std::string GEO_NAME;

    double geoidHeightM (double latitude_deg, double longitude_deg);
    double declination(float year, double latitude_deg, double longitude_deg, double altitude_m);

    void test();

protected:

    std::string current_projection_name_;

    std::unique_ptr<ProjectionManagerWidget> widget_;
    GDALRasterBand* egm96_band_{nullptr};
    //std::unique_ptr<GeographicLib::Geoid> geoid_;

    double egm96_band_inv_geo_transform_[6];

    GeographicLib::MagneticModel mag_model_;

    int egm96_band_width_{0};
    int egm96_band_height_{0};

    const double egm96_band_scale_ {0.003}; //from file, gdal too stupid
    const double egm96_band_offset_ {-108.0};

    GDALDataset* dataset_ {nullptr};

    std::map<std::string, std::unique_ptr<Projection>> projections_;

    virtual void checkSubConfigurables();

    unsigned int calculateRadarPlotPositions (std:: string dbcontent_name, std::shared_ptr<Buffer> buffer,
                                              NullableVector<double>& target_latitudes_vec,
                                              NullableVector<double>& target_longitudes_vec);
    unsigned int doXYPositionCalculations (std:: string dbcontent_name, std::shared_ptr<Buffer> buffer,
                                             NullableVector<double>& target_latitudes_vec,
                                             NullableVector<double>& target_longitudes_vec);
};

