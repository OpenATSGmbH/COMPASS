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

#include "colormap.h"
#include "colorscales.h"
#include "colorlegend.h"
#include "traced_assert.h"

#include "logger.h"

#include <QComboBox>
#include <QLabel>
#include <QGridLayout>

namespace
{
    /**
    */
    std::vector<double> interpFactors(size_t n)
    {
        if (n == 0)
            return {};
        if (n == 1)
            return { 0.0 };
        if (n == 2)
            return { 0.0, 1.0 };

        const size_t b = n - 1;

        std::vector<double> f(n);
        for (size_t i = 0; i < n; ++i)
            f[ i ] = (double)i / (double)b;

        return f;
    };

    /**
    */
    std::vector<double> valueFactors(size_t n)
    {
        if (n == 0)
            return {};
        if (n == 1)
            return { 0.0 };
        if (n == 2)
            return { 0.0, 1.0 };

        std::vector<double> f(n);
        f[ 0 ] = 0.0;

        const size_t b = n - 2;

        for (size_t i = 0; i < n - 1; ++i)
            f[ i + 1 ] = (double)i / (double)b;

        return f;
    };

    /**
    */
    size_t numColorsFromType(ColorMap::Type type, size_t n)
    {
        size_t num_colors = 0;
        if (type == ColorMap::Type::LinearSamples)
            num_colors = n;
        else if (type == ColorMap::Type::LinearRanges)
            num_colors = n + 2;
        else if (type == ColorMap::Type::Discrete)
            num_colors = n;
        else if (type == ColorMap::Type::Binary)
            num_colors = 2;

        traced_assert(num_colors >= 1);

        return num_colors;
    };

    /**
    */
    Eigen::Vector3d colorQt2Eigen(const QColor& color)
    {
        return Eigen::Vector3d(color.redF(), color.greenF(), color.blueF());
    }

    /**
    */
    QColor colorEigen2Qt(const Eigen::Vector3d& color)
    {
        return QColor(color[ 0 ] * 255, color[ 1 ] * 255, color[ 2 ] * 255);
    }

    /**
    */
    QColor mix(const QColor& color0, const QColor& color1, double t0, double t1)
    {
        return QColor(color0.red()   * t0 + color1.red()   * t1,
                      color0.green() * t0 + color1.green() * t1,
                      color0.blue()  * t0 + color1.blue()  * t1);
    }
}

/**
*/
ColorMap::ColorMap()
{
    //init special colors
    special_colors_.resize(NumSpecialColors);

    special_colors_[ SpecialColorSelected ] = Qt::yellow;
    special_colors_[ SpecialColorNull     ] = Qt::magenta;

    //init to valid map
    create(ColorScale::Green2Red, 5, ColorMap::Type::LinearSamples);
}

/**
*/
ColorMap::~ColorMap() = default;

/**
*/
bool ColorMap::valid() const
{
    return (n_colors_ >= 1);
}

/**
*/
bool ColorMap::canSampleValues() const
{
    return value_range_.has_value();
}

/**
*/
ColorMap::ColorScale ColorMap::colorScale() const
{
    return scale_;
}

/**
*/
size_t ColorMap::colorSteps() const
{
    return steps_;
}

/**
*/
size_t ColorMap::numColors() const
{
    return n_colors_;
}

/**
*/
const QColor& ColorMap::getColor(size_t idx) const
{
    return colors_.at(idx);
}

/**
*/
QColor ColorMap::sample(double t, ColorScale scale)
{
    auto c = colorscale::sample(t, scale);
    return QColor(c[ 0 ] * 255, c[ 1 ] * 255, c[ 2 ] * 255);
}

/**
*/
std::vector<QColor> ColorMap::sample(ColorScale scale, 
                                     size_t n)
{
    std::vector<QColor> colors(n);
    size_t i = 0;
    for (double t : interpFactors(n))
        colors[ i++ ] = sample(t, scale);

    return colors;
}

/**
*/
std::vector<QColor> ColorMap::sample(const std::vector<QColor>& samples, 
                                     size_t n)
{
    size_t n_samples = samples.size();
    traced_assert(n_samples >= 2);

    std::vector<Eigen::Vector3d> samples_01(n_samples);
    for (size_t i = 0; i < n_samples; ++i)
        samples_01[ i ] = colorQt2Eigen(samples[ i ]);

    std::vector<QColor> colors(n);
    size_t i = 0;
    for (double t : interpFactors(n))
        colors[ i++ ] = colorEigen2Qt(colorscale::sampleCustom(t, samples_01));

    return colors;
}

/**
*/
std::vector<QColor> ColorMap::sample(const QColor& color_min, 
                                     const QColor& color_max, 
                                     size_t n)
{
    return sample({ color_min, color_max }, n);
}

/**
*/
std::vector<QColor> ColorMap::sample(const QColor& color_min, 
                                     const QColor& color_mid, 
                                     const QColor& color_max,
                                     size_t n)
{
    return sample({ color_min, color_mid, color_max }, n);
}

/**
*/
void ColorMap::generateScaleImage(QImage& img, 
                                  ColorScale scale, 
                                  int w, 
                                  int h)
{
    img = QImage();
    
    if (w < 1 || h < 1)
        return;

    img = QImage(w, h, QImage::Format_ARGB32);

    auto factors = interpFactors(w);
    traced_assert(factors.size() == (size_t)w);

    //@TODO: optimize
    for (int x = 0; x < w; ++x)
    {
        const QColor c = sample(factors[ x ], scale);

        for (int y = 0; y < h; ++y)
            img.setPixelColor(x, y, c);
    }
}

/**
*/
QComboBox* ColorMap::generateScaleSelection(ColorScale scale_default)
{
    const int IconW = 100;
    const int IconH = 6;

    QComboBox* combo = new QComboBox;
    combo->setIconSize(QSize(IconW, IconH));

    std::map<QString, std::pair<QImage, ColorScale>> entries;

    for (auto scale : colorscale::UsedColorScales)
    {
        QString name = QString::fromStdString(colorscale::colorScale2String(scale));

        QImage img;
        generateScaleImage(img, scale, IconW, IconH);

        entries[ name ] = std::make_pair(img, scale);
    }

    int idx = -1;
    for (const auto& e : entries)
    {
        if (e.second.second == scale_default)
            idx = combo->count();

        combo->addItem(QIcon(QPixmap::fromImage(e.second.first)), e.first, QVariant((int)e.second.second));
    }

    if (idx >= 0)
        combo->setCurrentIndex(idx);

    return combo;
}

/**
*/
void ColorMap::create(const std::vector<QColor>& colors,
                      Type type,
                      ColorScale scale,
                      size_t steps,
                      const OValueRange& value_range)
{
    traced_assert(type != Type::Binary        || colors.size() == 2);
    traced_assert(type != Type::Discrete      || colors.size() >= 1);
    traced_assert(type != Type::LinearRanges  || colors.size() >= 3);
    traced_assert(type != Type::LinearSamples || colors.size() >= 1);

    traced_assert(!value_range.has_value() || value_range.value().first <= value_range.value().second);

    type_          = type;
    scale_         = scale;
    steps_         = steps;

    value_range_   = value_range;

    colors_        = colors;
    value_factors_ = type == Type::LinearRanges ? valueFactors(colors_.size()) : interpFactors(colors_.size());
    n_colors_      = colors_.size();
}

/**
*/
void ColorMap::create(const std::vector<QColor>& colors,
                      Type type,
                      const OValueRange& value_range)
{
    create(colors, type, ColorScale::Custom, colors.size(), value_range);
}

/**
*/
void ColorMap::create(const QColor& color_min, 
                      const QColor& color_max, 
                      size_t n,
                      Type type,
                      const OValueRange& value_range)
{
    traced_assert(n >= 1);

    auto colors = ColorMap::sample(color_min, color_max, numColorsFromType(type, n));
    create(colors, type, ColorScale::Custom, n, value_range);
}

/**
*/
void ColorMap::create(const QColor& color_min, 
                      const QColor& color_mid, 
                      const QColor& color_max, 
                      size_t n,
                      Type type,
                      const OValueRange& value_range)
{
    traced_assert(n >= 1);
    
    auto colors = ColorMap::sample(color_min, color_mid, color_max, numColorsFromType(type, n));
    create(colors, type, ColorScale::Custom, n, value_range);
}

/**
*/
void ColorMap::create(ColorScale scale,
                      size_t n,
                      Type type,
                      const OValueRange& value_range)
{
    traced_assert(n >= 1);

    auto colors = ColorMap::sample(scale, numColorsFromType(type, n));
    create(colors, type, scale, n, value_range);
}

/**
*/
void ColorMap::setSpecialColor(SpecialColor type, const QColor& color)
{
    special_colors_.at(type) = color;
}

/**
*/
const QColor& ColorMap::specialColor(SpecialColor type) const
{
    return special_colors_.at(type);
}

/**
 * Mapping of factors to a color index / value interval.
 * 
 * Example - Binary colormap:
 *     t <  0.5 ... color0
 *     t >= 0.5 ... color1
 * 
 * Example - Linear colormap: (e.g. n = 5 => yields 7 colors, 5 inner intervals, and 6 value factors)
 * 
 *     colors_        = [ color0, color1, color2, color3, color4, color5, color6 ]
 *     value_factors_ = [ 0.0, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0 ]
 * 
 *     t < 0.0        ... color0
 *     0.0 <= t < 0.2 ... color1
 *     0.2 <= t < 0.4 ... color2
 *     0.4 <= t < 0.6 ... color3
 *     0.6 <= t < 0.8 ... color4
 *     0.8 <= t < 1.0 ... color5
 *     t >= 1.0       ... color6
 * 
 *     => yielding idx: value_factors_[ idx ] <= t < value_factors_[ idx + 1 ]
 * 
 */
size_t ColorMap::indexFromFactor(double t) const
{
    //!discrete colormaps cannot be accessed by this method!
    traced_assert(type_ != Type::Discrete);

    //handle binary case separately
    if (type_ == Type::Binary)
    {
        traced_assert(n_colors_ == 2);
        return (t < 0.5 ? 0 : 1);
    }

    //linear colormap
    traced_assert(type_ == Type::LinearSamples || type_ == Type::LinearRanges);
    traced_assert(type_ != Type::LinearRanges  || n_colors_ >= 3);
    traced_assert(type_ != Type::LinearSamples || n_colors_ >= 1);

    size_t idx;

    if (type_ == Type::LinearRanges)
    {
        //handle maximum colors
        if (t < 0.0)
            return 0;
        if (t >= 1.0)
            return n_colors_ - 1;

        //t in [0, 1)
        const size_t n_inner = n_colors_ - 2;
        idx                  = (size_t)std::floor(t * n_inner) + 1;

        if (idx > n_colors_ - 2)
        {
            logerr << "idx " << idx << " > " << n_colors_ - 2
                   << " for t " << t;
            idx = n_colors_ - 2;
        }

        traced_assert(idx <= n_colors_ - 2);
    }
    else // Type::LinearSamples
    {
        if (n_colors_ == 1)
            return 0;

        const double step  = 1.0 / (n_colors_ - 1);
        const double hstep = step * 0.5;

        if (t < hstep)
            return 0;
        if (t >= 1.0 - hstep)
            return n_colors_ - 1;

        const size_t n_inner   = n_colors_ - 2;
        const size_t idx_inner = (size_t)std::floor((t - hstep) * n_inner);
        idx                    = 1 + idx_inner;

        if (idx > n_colors_ - 2)
        {
            logerr << "idx " << idx << " > " << n_colors_ - 2
                   << " for t " << t;
            idx = n_colors_ - 2;
        }

        traced_assert(idx <= n_colors_ - 2);
    }

    return idx;
}

/**
*/
QColor ColorMap::sample(double t) const
{
    size_t idx = indexFromFactor(t);

    return colors_.at(idx);
}

/**
*/
QColor ColorMap::sampleValue(double v) const
{
    traced_assert(canSampleValues());

    auto range = activeRange();

    const double vrange = range.second - range.first;
    if (vrange < 1e-12)
        return colors_.front();
    
    const double t = (v - range.first) / vrange;
    
    return sample(t);
}

/**
*/
ColorMap::ValueRange ColorMap::activeRange() const
{
    traced_assert(canSampleValues());

    double vmin = value_range_.value().first;
    double vmax = value_range_.value().second;

    if (sample_values_symm_)
    {
        const double vmax_abs = std::max(std::fabs(vmin), std::fabs(vmax));

        vmin = -vmax_abs;
        vmax =  vmax_abs;
    }

    return std::make_pair(vmin, vmax);
}

/**
*/
ColorLegend ColorMap::colorLegend(bool add_sel_color,
                                  bool add_null_color,
                                  const ValueDecorator& decorator) const
{
    if (!valid() || !canSampleValues())
        return ColorLegend();

    ValueDecorator active_decorator = decorator;
    if (!active_decorator)
        active_decorator = [ & ] (double value) { return std::to_string(value); };

    ColorLegend legend;

    auto range = activeRange();

    const double vrange = range.second - range.first;

    if (vrange < 1e-12 || numColors() == 1)
    {
        if (type_ == Type::LinearRanges)
            legend.addEntry(colors_.front(), " = " + active_decorator(range.first));
        else
            legend.addEntry(colors_.front(), active_decorator(range.first));
    }
    else
    {
        if (type_ == Type::LinearRanges)
        {
            legend.addEntry(colors_.front(), "< " + active_decorator(range.first));

            for (size_t i = 1; i < n_colors_ - 1; ++i)
            {
                const double v0 = range.first + value_factors_[ i     ] * vrange;
                const double v1 = range.first + value_factors_[ i + 1 ] * vrange;

                legend.addEntry(colors_[ i ], active_decorator(v0) + " - " + active_decorator(v1));
            }

            legend.addEntry(colors_.back(), ">= " + active_decorator(range.second));
        }
        else
        {
            for (size_t i = 0; i < n_colors_; ++i)
            {
                const double v = range.first + value_factors_[ i ] * vrange;
                legend.addEntry(colors_[ i ], active_decorator(v));
            }
        }
    }

    if (add_sel_color)
        legend.addEntry(specialColor(SpecialColor::SpecialColorSelected), "Selected");

    if (add_null_color)
        legend.addEntry(specialColor(SpecialColor::SpecialColorNull), "NULL");

    return legend;
}
