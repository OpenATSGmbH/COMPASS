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
#include "imanager.h"
#include "buffer.h"

class IDBVariableResolver;

namespace context
{
    class DBContextManager;
    class FFT;
}

#include "gdal_priv.h"

// fix stupid gdal constant
#ifdef PACKAGE_VERSION
#undef PACKAGE_VERSION
#endif

#include <GeographicLib/MagneticModel.hpp>

#include <boost/optional.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <tuple>

class ProjectionManagerWidget;
class Projection;
class GeoProjection;

class ProjectionManager : public Configurable, public IManager
{
public:
    ProjectionManager(nlohmann::json& config, Configurable* parent);
    virtual ~ProjectionManager();

    ProjectionManagerWidget* widget();
    void deleteWidget();

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

    /// Position (latitude, longitude) at which radar ds_id plots a delayed-reply FFT:
    /// azimuth of the FFT site, slant range of the site plus the FFT range delay.
    /// None if the FFT has no range delay or position, or the radar has no coordinate
    /// system. Cached, thread-safe.
    boost::optional<std::pair<double, double>> fftExpectedPosition(unsigned int ds_id, const context::FFT& fft);

    /// FFT check of a radar plot of data source ds_id: as DBContextManager::isFromFFT, and
    /// FFTs with a range delay also match at their expected position. Returns {is_fft, altitude_ft}.
    std::pair<bool, float> isFromFFT(unsigned int ds_id, double latitude_deg, double longitude_deg,
                                     boost::optional<unsigned int> mode_s_address, bool ignore_mode_s,
                                     boost::optional<unsigned int> mode_a_code,
                                     boost::optional<float> mode_c_code);

    double geoidHeightM (double latitude_deg, double longitude_deg);
    double declination(float year, double latitude_deg, double longitude_deg, double altitude_m);

    void test();

    // peers, set by the composition root (the context manager is created after this manager)
    void setManagers(context::DBContextManager& ctx_man) { ctx_man_ = &ctx_man; }
    bool checkWired() const override;
    context::DBContextManager& dbContextManager();

    void varResolver(IDBVariableResolver& resolver) { var_resolver_ = &resolver; }
    IDBVariableResolver& varResolver() const { return *var_resolver_; }

protected:
    GenerateResult generateSubConfigurable_impl(nlohmann::json& child_json) override;

    context::DBContextManager* ctx_man_      = nullptr; // peer, set by the composition root
    IDBVariableResolver*       var_resolver_ = nullptr;

    // fftExpectedPosition cache, key: ds_id, radar lat/lon/alt, FFT lat/lon/alt, range delay
    // (values in the key, so edits of the radar or the FFT give a new entry)
    using FFTExpectedKey = std::tuple<unsigned int, double, double, double, double, double, double, double>;
    std::map<FFTExpectedKey, boost::optional<std::pair<double, double>>> fft_expected_cache_;
    std::mutex fft_expected_mutex_;

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

