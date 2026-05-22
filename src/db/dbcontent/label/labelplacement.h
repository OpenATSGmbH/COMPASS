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

#include "labelplacement_defs.h"

#include <string>
#include <vector>
#include <map>

#include <QRectF>
#include <QColor>

#include <boost/optional.hpp>

class QImage;

/**
 */
class LabelPlacementEngine
{
public:
    typedef label_placement::Method   Method;
    typedef label_placement::Label    Label;
    typedef label_placement::Settings Settings;
    
    /**
     * Configuration for test/debug output
     */
    struct TestConfig
    { 
        typedef std::function<void(double&, double&, bool)>             ScreenTransform;
        typedef std::function<void(double&, double&, double&, double&)> ScreenTransformBBox;

        int width  = 1024;
        int height = 768;

        int num_objects = 100;
        int interval_ms = 1000;

        double speed      = 0.01;
        double radius     = 0.005;

        double label_w    = 0.04;
        double label_h    = 0.015;
        double label_offs = 0.015;

        bool flip_y        = false;
        bool avoid_anchors = false;
        bool avoid_roi     = false;

        mutable QRectF              roi;
        mutable ScreenTransform     screen_transform;
        mutable ScreenTransformBBox screen_transform_bbox;
    };

    /**
     * Test label for test/debug output
     */
    struct TestLabel
    {
        TestLabel() : color(255, 255, 255) {}

        std::string txt;                //test label screen text

        boost::optional<double> x_init; //test label init x position for movement visualization
        boost::optional<double> y_init; //test label init y position for movement visualization

        double dirx;  //x movement direction
        double diry;  //y movement direction
        double speed; //movement speed

        Label  label;  //test label data
        QColor color;  //test label color
    };

    LabelPlacementEngine();
    virtual ~LabelPlacementEngine() = default;

    void addLabel(const std::string& id, 
                  double x_anchor, 
                  double y_anchor,
                  double w,
                  double h,
                  double* x_init = nullptr, 
                  double* y_init = nullptr,
                  double* x_ref = nullptr,
                  double* y_ref = nullptr);
    const Label& getLabel(size_t idx) const;
    const Label* getLabel(const std::string& id) const;
    void removeLabel(size_t idx);
    void removeLabel(const std::string& id);
    void clearLabels();

    bool placeLabels();

    Settings& settings() { return settings_; } 

    void showData(const TestConfig& test_config) const;
    void runTest(const TestConfig& test_config) const;

private:
    void revertPlacements();

    void computeScreenTransform(const TestConfig& test_config, 
                                const std::vector<TestLabel>& test_labels) const;
    void runTest(const std::vector<TestLabel>& test_labels,
                 const TestConfig& test_config) const;
    void renderTestFrame(QImage& img, 
                         const std::vector<TestLabel>& labels,
                         const TestConfig& test_config) const;

    std::vector<Label>            labels_;
    std::map<std::string, size_t> label_map_;

    Settings                      settings_;
};
