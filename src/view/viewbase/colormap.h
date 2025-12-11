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

#include "colormap_defs.h"

#include <vector>

#include <boost/optional.hpp>

#include <QColor>

class QComboBox;
class QImage;
class ColorLegend;

/**
*/
class ColorMap 
{
public:
    typedef colorscale::ColorScale      ColorScale;
    typedef std::pair<double, double>   ValueRange;
    typedef boost::optional<ValueRange> OValueRange;

    typedef std::function<std::string(double)> ValueDecorator;

    enum class Type
    {
        LinearSamples = 0,
        LinearRanges,
        Discrete,
        Binary
    };

    enum SpecialColor
    {
        SpecialColorSelected = 0,
        SpecialColorNull,

        NumSpecialColors
    };

    ColorMap();
    virtual ~ColorMap();

    static QColor sample(double t, 
                         ColorScale scale);
    static std::vector<QColor> sample(ColorScale scale, 
                                      size_t n);
    static QColor sample(double t, 
                         const std::vector<QColor>& samples);
    static std::vector<QColor> sample(const std::vector<QColor>& samples, 
                                      size_t n);
    static QColor sample(double t, 
                         const QColor& color_min, 
                         const QColor& color_max);
    static std::vector<QColor> sample(const QColor& color_min, 
                                      const QColor& color_max, 
                                      size_t n);
    static QColor sample(double t,
                         const QColor& color_min, 
                         const QColor& color_mid, 
                         const QColor& color_max);
    static std::vector<QColor> sample(const QColor& color_min, 
                                      const QColor& color_mid, 
                                      const QColor& color_max,
                                      size_t n);

    static void generateScaleImage(QImage& img, 
                                   ColorScale scale, 
                                   int w, 
                                   int h);
    static QComboBox* generateScaleSelection(ColorScale scale_default = ColorScale::Green2Red);

    bool valid() const;
    bool canSampleValues() const;

    ColorScale colorScale() const;
    size_t colorSteps() const;

    size_t numColors() const;
    const QColor& getColor(size_t idx) const;
    const std::vector<QColor>& getColors() const { return colors_; }

    const OValueRange& valueRange() const { return value_range_; }

    void create(const std::vector<QColor>& colors,
                Type type = Type::LinearSamples,
                const OValueRange& value_range = OValueRange());
    void create(const QColor& color_min,
                const QColor& color_max,
                size_t n,
                Type type = Type::LinearSamples,
                const OValueRange& value_range = OValueRange());
    void create(const QColor& color_min,
                const QColor& color_mid,
                const QColor& color_max,
                size_t n,
                Type type = Type::LinearSamples,
                const OValueRange& value_range = OValueRange());
    void create(ColorScale scale, 
                size_t n,
                Type type = Type::LinearSamples,
                const OValueRange& value_range = OValueRange());
    
    void setSpecialColor(SpecialColor type, const QColor& color);
    const QColor& specialColor(SpecialColor type) const;
    
    QColor sample(double t) const;
    QColor sampleValue(double v) const;

    ColorLegend colorLegend(bool add_sel_color = true,
                            bool add_null_color = true,
                            const ValueDecorator& decorator = ValueDecorator()) const;
private:
    void create(const std::vector<QColor>& colors,
                Type type,
                ColorScale scale,
                size_t steps,
                const OValueRange& value_range);

    size_t indexFromFactor(double t) const;
    ValueRange activeRange() const;

    Type       type_  = Type::LinearSamples;
    ColorScale scale_ = ColorScale::Custom;
    size_t     steps_ = 0;

    std::vector<QColor> colors_;
    std::vector<double> value_factors_;
    size_t              n_colors_ = 0;

    std::vector<QColor> special_colors_;
    OValueRange         value_range_;
    bool                sample_values_symm_ = false;
};
