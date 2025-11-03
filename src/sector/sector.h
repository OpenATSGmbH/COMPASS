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

#include "json_fwd.hpp"

#include <QColor>
#include <QRectF>
#include <QImage>

#include <memory>

#include <boost/optional.hpp>

class DBInterface;

namespace dbContent {
class TargetPosition;
}

class OGRPolygon;

/**
 * Fast sector inside test via image discretization.
 * 
 * Generates an discrete image map representation of the input polygon and checks this 
 * representation for definitely inside, definitely outside and dubious points.
 * Dubious points reside near the polygon border and can be checked later on with a
 * more accurate geometrical inside test. 
 */
class SectorInsideTest
{
public:
    enum class CheckResult
    {
        Inside = 0,
        Outside,
        Border
    };

    SectorInsideTest();
    SectorInsideTest(const std::vector<std::pair<double,double>>& points, 
                     double xmin, 
                     double ymin, 
                     double xmax, 
                     double ymax);
    virtual ~SectorInsideTest();

    CheckResult isInside(double x, double y) const;
    CheckResult isInside(double x, double y, double delta) const;

    bool isValid() const;

    const QImage& image() const { return img_; }

private:
    static const int ImageSize    = 1000; //maximum image map extent (data region aspect will be preserved)
    static const int ImgBorder    = 5;    //border added to the image map for safety
    static const int BorderRegion = 7;    //width of the polygon border region (= dubious region)

    void map2Image(double& xm, double& ym, double x, double y) const;

    void create(const std::vector<std::pair<double,double>>& points, 
                double xmin, 
                double ymin, 
                double xmax, 
                double ymax);

    QImage img_;
    double xmin_ = 0;
    double ymin_ = 0;
    double tx_   = 1;
    double ty_   = 1;
};

/**
 * A basic sector.
 * - contains an id, a name and a name of the layer it belongs to
 * - contains a describing polygon in the plane
 * - contains optional minimum and maximum altitude
 * - can be inclusive or exclusive
 * - provides means for a (fast) inside check (planar, vertical, both)
 * - provides bounds of the polygonal data it stores
 * - provides means to serialize the sector to the database
 */
class Sector
{
public:
    enum InsideCheckType
    {
        XY = 0,
        Z,
        ZMinOnly,
        XYZ
    };

    Sector(unsigned int id, 
           const std::string& name, 
           const std::string& layer_name,
           bool serialize,
           bool exclusion_sector,
           QColor color, 
           std::vector<std::pair<double,double>> points);
    Sector(unsigned int id,
           const std::string& name, 
           const std::string& layer_name,
           bool serialize);
    virtual ~Sector();

    unsigned int id() const;

    std::string name() const;
    void name(const std::string& name);

    std::string layerName() const;
    void layerName(const std::string& layer_name);

    bool readJSON(const std::string& json_str);
    bool readJSON(const nlohmann::json& json_obj);
    nlohmann::json jsonData() const;
    std::string jsonDataStr() const;

    unsigned int size () { return points_.size(); }

    const std::vector<std::pair<double, double>>& points() const;

    std::string colorStr();
    void colorStr(std::string value);
    void removeColorStr();

    bool isExclusionSector() const { return exclusion_sector_; }
    virtual void exclude(bool ok);

    bool hasMinimumAltitude() const;
    double minimumAltitude() const;
    virtual void setMinimumAltitude(double value);
    virtual void removeMinimumAltitude();

    bool hasMaximumAltitude() const;
    double maximumAltitude() const;
    virtual void setMaximumAltitude(double value);
    virtual void removeMaximumAltitude();

    bool serializeSector() const;
    void serializeSector(bool ok);
    void save();
    
    virtual bool isInside(const dbContent::TargetPosition& pos, 
                          bool has_ground_bit, 
                          bool ground_bit_set,
                          InsideCheckType check_type = InsideCheckType::XYZ) const;
    bool isInside(double latitude, double longitude, double delta_deg) const;

    std::pair<double, double> getMinMaxLatitude() const;
    std::pair<double, double> getMinMaxLongitude() const;

    void createFastInsideTest() const;

protected:
    void createPolygon();

    virtual bool readJSON_impl(const nlohmann::json& json_obj) { return true; };
    virtual void writeJSON_impl(nlohmann::json& json_obj) const {};

    unsigned int id_;
    std::string  name_;
    std::string  layer_name_;

    bool serialize_        = false;
    bool exclusion_sector_ = false; //@TODO: maybe describe this e.g. as sector flags to make sectors more versatile?

    std::string  color_str_;

    std::vector<std::pair<double,double>> points_;

    boost::optional<double> min_altitude_;
    boost::optional<double> max_altitude_;

    boost::optional<double> lat_min_;
    boost::optional<double> lat_max_;
    boost::optional<double> lon_min_;
    boost::optional<double> lon_max_;

    mutable boost::optional<SectorInsideTest> inside_test_;

    std::unique_ptr<OGRPolygon> ogr_polygon_;
};
