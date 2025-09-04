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

#include "sector.h"
#include "compass.h"
#include "evaluationmanager.h"
#include "dbcontent/target/targetposition.h"

#include "traced_assert.h"
#include <cmath>

#include <ogr_geometry.h>

#include <QPainter>

using namespace nlohmann;
using namespace std;

const string default_color{"#AAAAAA"};

/***********************************************************************************
 * SectorInsideTest
 ***********************************************************************************/

SectorInsideTest::SectorInsideTest() = default;

SectorInsideTest::SectorInsideTest(const std::vector<std::pair<double,double>>& points, 
                                   double xmin, 
                                   double ymin, 
                                   double xmax, 
                                   double ymax)

{
    create(points, xmin, ymin, xmax, ymax);
}

SectorInsideTest::~SectorInsideTest() = default;

void SectorInsideTest::map2Image(double& xm, double& ym, double x, double y) const
{
    xm = ImgBorder + BorderRegion + (x - xmin_) * tx_;
    ym = ImgBorder + BorderRegion + (y - ymin_) * ty_;
}

void SectorInsideTest::create(const std::vector<std::pair<double,double>>& points, 
                              double xmin, 
                              double ymin, 
                              double xmax, 
                              double ymax)
{
    img_ = QImage();

    //check if bounds are even valid
    if (!std::isfinite(xmin) ||
        !std::isfinite(ymin) ||
        !std::isfinite(xmax) ||
        !std::isfinite(ymax) ||
        xmax <= xmin || 
        ymax <= ymin)
        return;

    //remember origin for later mapping
    xmin_ = xmin;
    ymin_ = ymin;

    double dx     = xmax - xmin;
    double dy     = ymax - ymin;
    double aspect = dx / dy;

    //@TODO: thresholds realistic?
    if (dx < 1e-06 || 
        dy < 1e-06 || 
        !std::isfinite(aspect) || 
        std::isinf(aspect) || 
        aspect < 1e-06 || 
        aspect > 1e+06)
        return;

    //generate image area of data aspect with maximum extent of 'ImageSize'
    int area_w = (int)std::max(1.0, aspect >= 1 ? ImageSize : std::ceil(ImageSize * aspect));
    int area_h = (int)std::max(1.0, aspect >= 1 ? std::ceil(ImageSize / aspect) : ImageSize);

    //data -> image mapping factors
    tx_ = area_w / dx;
    ty_ = area_h / dy;

    //we add some border for final image size
    int imgw = area_w + 2 * ImgBorder + 2 * BorderRegion;
    int imgh = area_h + 2 * ImgBorder + 2 * BorderRegion;

    img_ = QImage(imgw , imgh, QImage::Format_Grayscale8);
    img_.fill(Qt::black); //initially fill with black color (= all outside)

    size_t n = points.size();

    //convert points to image space
    QPolygonF poly(n);
    double xm, ym;

    for (size_t i = 0; i < n; ++i)
    {
        map2Image(xm, ym, points[ i ].first, points[ i ].second);
        poly[ i ] = QPointF(xm, ym);
    }

    //!disable antialiasing!
    QPainter p(&img_);
    p.setRenderHint(QPainter::RenderHint::Antialiasing, false);

    QPen pen;
    QBrush brush;

    pen.setCapStyle(Qt::PenCapStyle::RoundCap);
    pen.setJoinStyle(Qt::PenJoinStyle::RoundJoin);

    //draw inside region (= region where we assume inside for sure)
    {
        pen.setColor(Qt::white);
        pen.setWidth(1);

        brush.setColor(Qt::white);
        brush.setStyle(Qt::BrushStyle::SolidPattern);

        p.setPen(pen);
        p.setBrush(brush);
        p.drawPolygon(poly);
    }
    
    //draw border region (= region where we need an accurate - geometrical - inside check)
    {
        pen.setColor(Qt::gray);
        pen.setWidth(BorderRegion); //thicker pen for border region
    
        p.setPen(pen);
        p.setBrush(Qt::NoBrush);
        p.drawPolygon(poly);

#if 0
        //[DEBUG]
        pen.setColor(Qt::black);
        pen.setWidth(1);

        p.setPen(pen);
        p.drawPolygon(poly);
#endif
    }
}

namespace
{
    SectorInsideTest::CheckResult whereAmI(const QImage& img, int x, int y)
    {
        //when no accurate statement is possible we assume border region (= detailed check) 
        if (img.isNull() || x < 0 || y < 0 || x >= img.width() || y >= img.height())
            return SectorInsideTest::CheckResult::Border;

        //check color code at pixel
        auto rgb = img.pixel(x, y);

        if (rgb == Qt::gray)
            return SectorInsideTest::CheckResult::Border;
        else if (rgb == Qt::white)
            return SectorInsideTest::CheckResult::Inside;
        else if (rgb == Qt::black)
            return SectorInsideTest::CheckResult::Outside;

        //again: when in doubt assume border region
        return SectorInsideTest::CheckResult::Border;
    }
}

SectorInsideTest::CheckResult SectorInsideTest::isInside(double x, double y) const
{
    if (!isValid())
        return CheckResult::Border;

    //map to image area
    double xm, ym;
    map2Image(xm, ym, x, y);

    //round down to pixel
    int x0 = (int)xm;
    int y0 = (int)ym;

    //check 4-nh for map code
    auto check00 = whereAmI(img_, x0    , y0    );
    auto check10 = whereAmI(img_, x0 + 1, y0    );
    auto check01 = whereAmI(img_, x0    , y0 + 1);
    auto check11 = whereAmI(img_, x0 + 1, y0 + 1);

    //all outside? => assume outside
    if (check00 == CheckResult::Outside &&
        check10 == CheckResult::Outside &&
        check01 == CheckResult::Outside &&
        check11 == CheckResult::Outside)
        return CheckResult::Outside;

    //all inside? => assume inside
    if (check00 == CheckResult::Inside &&
        check10 == CheckResult::Inside &&
        check01 == CheckResult::Inside &&
        check11 == CheckResult::Inside)
        return CheckResult::Inside;

    //rest => assume on border
    return CheckResult::Border;
}

bool SectorInsideTest::isValid() const
{
    return !img_.isNull();
}

SectorInsideTest::CheckResult SectorInsideTest::isInside(double x, double y, double delta) const
{
    traced_assert(isValid());

    // First check the point itself
    auto result = isInside(x, y);
    
    // If already inside, return inside
    if (result == CheckResult::Inside)
    {
        loginf << "true inside";
        return CheckResult::Inside;
    }
    
    // If outside or border, check within delta distance
    // Sample points around the original point within delta distance
    const int num_samples = 16;
    const double angle_step = 2.0 * M_PI / num_samples;
    
    for (int i = 0; i < num_samples; ++i)
    {
        double angle = i * angle_step;
        double test_x = x + delta * std::cos(angle);
        double test_y = y + delta * std::sin(angle);
        
        auto test_result = isInside(test_x, test_y);
        if (test_result == CheckResult::Inside)
        {
            loginf << "second inside";
            return CheckResult::Inside;
        }
    }
    
    // Also check at half delta distance
    for (int i = 0; i < num_samples; ++i)
    {
        double angle = i * angle_step;
        double test_x = x + (delta * 0.5) * std::cos(angle);
        double test_y = y + (delta * 0.5) * std::sin(angle);
        
        auto test_result = isInside(test_x, test_y);
        if (test_result == CheckResult::Inside)
        {
            loginf << "thirst inside";
            return CheckResult::Inside;
        }
    }
    
    loginf << "not inside";
    return result; // Return original result if no nearby point is inside
}

/***********************************************************************************
 * Sector
 ***********************************************************************************/

Sector::Sector(unsigned int id, 
               const std::string& name, 
               const std::string& layer_name,
               bool serialize,
               bool exclusion_sector,
               QColor color, 
               std::vector<std::pair<double,double>> points)
:   id_              (id)
,   name_            (name)
,   layer_name_      (layer_name)
,   serialize_       (serialize)
,   exclusion_sector_(exclusion_sector)
,   color_str_       (color.name().toStdString())
,   points_          (points)
{
    createPolygon();
}

Sector::Sector(unsigned int id, 
               const std::string& name, 
               const std::string& layer_name,
               bool serialize)
:   id_        (id)
,   name_      (name)
,   layer_name_(layer_name)
,   serialize_ (serialize)
{
}

Sector::~Sector() = default;

bool Sector::readJSON(const std::string& json_str)
{
    auto json_obj = nlohmann::json::parse(json_str);

    return readJSON(json_obj);
}

bool Sector::readJSON(const nlohmann::json& j)
{
    min_altitude_.reset();
    max_altitude_.reset();
    
    traced_assert(j.contains("id"));
    traced_assert(j.at("id") == id_);

    traced_assert(j.contains("name"));
    traced_assert(j.at("name") == name_);

    traced_assert(j.contains("layer_name"));
    traced_assert(j.at("layer_name") == layer_name_);

    if (j.contains("min_altitude"))
        min_altitude_ = j.at("min_altitude");

    if (j.contains("max_altitude"))
        max_altitude_ = j.at("max_altitude");

    if (j.contains("exclude"))
        exclusion_sector_ = j.at("exclude");
    else
        exclusion_sector_ = false;

    traced_assert(j.contains("points"));
    const json& points = j.at("points");
    traced_assert(points.is_array());

    for (const json& point_it : points.get<json::array_t>())
    {
        traced_assert(point_it.is_array());
        traced_assert(point_it.size() == 2);
        points_.push_back({point_it[0], point_it[1]});
    }

    if (!j.contains("color_str"))
        color_str_ = default_color;
    else
        color_str_ = j.at("color_str");

    if (!readJSON_impl(j))
        return false;

    createPolygon();

    return true;
}

nlohmann::json Sector::jsonData() const
{
    json j = json::object();

    j["id"] = id_;
    j["name"] = name_;
    j["layer_name"] = layer_name_;

    if (min_altitude_.has_value())
        j["min_altitude"] = min_altitude_.value();

    if (max_altitude_.has_value())
        j["max_altitude"] = max_altitude_.value();

    j["exclude"] = exclusion_sector_;

    j["points"] = json::array();

    unsigned int cnt = 0;
    for (auto& p_it : points_)
    {
        j["points"][cnt] = json::array();;
        j["points"][cnt][0] = p_it.first;
        j["points"][cnt][1] = p_it.second;
        ++cnt;
    }

    j["color_str"] = color_str_;

    writeJSON_impl(j);

    return j;
}

std::string Sector::jsonDataStr() const
{
    return jsonData().dump();
}

std::string Sector::name() const
{
    return name_;
}

std::string Sector::layerName() const
{
    return layer_name_;
}

const std::vector<std::pair<double, double>>& Sector::points() const
{
    return points_;
}

std::string Sector::colorStr()
{
    return color_str_;
}

void Sector::colorStr(std::string value)
{
    loginf << "'" << value << "'";

    color_str_ = value;

    save();
}

void Sector::removeColorStr()
{
    color_str_ = default_color;

    save();
}

void Sector::exclude(bool ok)
{
    exclusion_sector_ = ok;

    save();
}


unsigned int Sector::id() const
{
    return id_;
}

bool Sector::hasMinimumAltitude() const
{
    return min_altitude_.has_value();
}

double Sector::minimumAltitude() const
{
    traced_assert(min_altitude_.has_value());
    return min_altitude_.value();
}

/**
*/
void Sector::setMinimumAltitude(double value)
{
    loginf << "start" << value << "";

    min_altitude_ = value;

    save();
}

/**
*/
void Sector::removeMinimumAltitude()
{
    loginf;

    min_altitude_.reset();

    save();
}

bool Sector::hasMaximumAltitude() const
{
    return max_altitude_.has_value();
}

double Sector::maximumAltitude() const
{
    traced_assert(max_altitude_.has_value());
    return max_altitude_.value();
}

/**
*/
void Sector::setMaximumAltitude(double value)
{
    loginf << "start" << value << "";

    max_altitude_ = value;

    save();
}

/**
*/
void Sector::removeMaximumAltitude()
{
    loginf;

    max_altitude_.reset();

    save();
}

void Sector::name(const std::string& name)
{
    loginf << "'" << name << "'";

    name_ = name;

    save();
}

void Sector::layerName(const std::string& layer_name)
{
    loginf << "'" << layer_name << "'";

    EvaluationManager& eval_man = COMPASS::instance().evaluationManager();

    string old_layer_name = layer_name_;
    layer_name_ = layer_name;

    eval_man.moveSector(id_, old_layer_name, layer_name); // moves and saves
}

bool Sector::serializeSector() const
{
    return serialize_;
}

void Sector::serializeSector(bool ok)
{
    serialize_ = ok;
}

void Sector::save()
{
    if (serialize_)
    {
        EvaluationManager& eval_man = COMPASS::instance().evaluationManager();
        eval_man.saveSector(id_);
    }
}

bool Sector::isInside(const dbContent::TargetPosition& pos,
                      bool has_ground_bit, 
                      bool ground_bit_set,
                      InsideCheckType check_type) const
{
    if (check_type == InsideCheckType::XYZ ||
        check_type == InsideCheckType::Z ||
        check_type == InsideCheckType::ZMinOnly)
    {
        //check altitude range
        if (pos.has_altitude_)
        {
            //check min
            if (min_altitude_.has_value() && pos.altitude_ < min_altitude_.value())
                return false;

            //check max
            if (check_type != InsideCheckType::ZMinOnly && max_altitude_.has_value() && pos.altitude_ > max_altitude_.value())
                return false;
        }

        //check gb against min
        if (has_ground_bit && ground_bit_set && min_altitude_.has_value())
            return false;
    }

    if (check_type == InsideCheckType::XYZ ||
        check_type == InsideCheckType::XY)
    {
        //check bounding rect
        if ((lat_min_.has_value() && pos.latitude_  < lat_min_.value()) ||
            (lat_max_.has_value() && pos.latitude_  > lat_max_.value()) ||
            (lon_min_.has_value() && pos.longitude_ < lon_min_.value()) ||
            (lon_max_.has_value() && pos.longitude_ > lon_max_.value()))
        {
            return false;
        }

        //check fast inside test if available
        bool check_geom = true;
        if (inside_test_.has_value())
        {
            auto res = inside_test_->isInside(pos.latitude_, pos.longitude_);
            if (res == SectorInsideTest::CheckResult::Outside)
                return false;
            else if (res == SectorInsideTest::CheckResult::Inside)
                check_geom = false; //geometrical check not needed
            else 
                check_geom = true;  //check yielded 'SectorInsideTest::CheckResult::Border' => check polygon geometrically
        }

        //check poly
        if (check_geom)
        {
            OGRPoint ogr_pos (pos.latitude_, pos.longitude_);
            if (!ogr_polygon_->Contains(&ogr_pos))
                return false;
        }
    }

    return true;
}

bool Sector::isInside(double latitude, double longitude, double delta_deg) const
{
    OGRPoint ogr_pos(latitude, longitude);
    if (ogr_polygon_->Contains(&ogr_pos))
        return true;

    // If outside or border, check within delta distance
    // Sample points around the original point within delta distance
    const int num_samples = 16;
    const double angle_step = 2.0 * M_PI / num_samples;

    double angle, test_x, test_y;

    for (int i = 0; i < num_samples; ++i)
    {
        angle = i * angle_step;
        test_x = latitude + delta_deg * std::cos(angle);
        test_y = longitude + delta_deg * std::sin(angle);
        
        ogr_pos = {test_x, test_y};

        if (ogr_polygon_->Contains(&ogr_pos))
            return true;
    }
    
    // Also check at half delta distance
    for (int i = 0; i < num_samples; ++i)
    {
        angle = i * angle_step;
        test_x = latitude + (delta_deg * 0.5) * std::cos(angle);
        test_y = longitude + (delta_deg * 0.5) * std::sin(angle);
        
        ogr_pos = {test_x, test_y};

        if (ogr_polygon_->Contains(&ogr_pos))
            return true;
    }

    // final: check distance to polygon points (for small geometry)

    for (auto& point_it : points_)
    {
        if (sqrt(pow(point_it.first-latitude, 2)+pow(point_it.second-longitude, 2)) < delta_deg)
            return true;
    }

    return false;
}

std::pair<double, double> Sector::getMinMaxLatitude() const
{
    traced_assert(lat_min_.has_value() && lat_max_.has_value());
    
    return {lat_min_.value(), lat_max_.value()};
}

std::pair<double, double> Sector::getMinMaxLongitude() const
{
    traced_assert(lon_min_.has_value() && lon_max_.has_value());

    return {lon_min_.value(), lon_max_.value()};
}

void Sector::createPolygon()
{
    //reset bounding rect
    lat_min_.reset();
    lat_max_.reset();
    lon_min_.reset();
    lon_max_.reset();

    ogr_polygon_.reset(new OGRPolygon());
    //ogr_polygon_->addRingDirectly()
    OGRLinearRing* ring = new OGRLinearRing();

    for (auto& point_it : points_)
    {
        ring->addPoint(point_it.first, point_it.second);

        //keep track of bounding rect
        if (!lat_min_.has_value() || point_it.first  < lat_min_.value()) lat_min_ = point_it.first;
        if (!lon_min_.has_value() || point_it.second < lon_min_.value()) lon_min_ = point_it.second;
        if (!lat_max_.has_value() || point_it.first  > lat_max_.value()) lat_max_ = point_it.first;
        if (!lon_max_.has_value() || point_it.second > lon_max_.value()) lon_max_ = point_it.second;
    }

    if (*points_.begin() != *points_.rbegin())
        ring->addPoint(points_.begin()->first, points_.begin()->second); // close if not not closed

    ogr_polygon_->addRingDirectly(ring);
}

void Sector::createFastInsideTest() const
{
    traced_assert(lat_min_.has_value() && lat_max_.has_value());
    traced_assert(lon_min_.has_value() && lon_max_.has_value());

    //create sector inside test
    inside_test_ = SectorInsideTest(points_, lat_min_.value(), lon_min_.value(), lat_max_.value(), lon_max_.value());

#if 0
    //[DEBUG]
    if (inside_test_->isValid())
        inside_test_->image().save("/home/mcphatty/sector" + QString::number(id_) + "_inside_test.png");
#endif

    //dump if invalid (numerical issues etc.)
    if (!inside_test_->isValid())
        inside_test_.reset();
}
