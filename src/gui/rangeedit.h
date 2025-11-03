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

//#include "property/property.h"

#include <iostream>

#include <QWidget>
#include <QString>
#include <QVariant>

class RangeSlider;

class QLineEdit;

namespace rangeedit
{
    enum class RangeType { Min = 0, Max };

    /**
     * Represents a certain range between two numerical limit values.
     * limit min < limit_max
     * limit_min <= range_min <= range_max <= limit_max
     */
    template<typename T>
    class Range
    {
    public:
        Range() : limit_min_(0), limit_max_(0), range_min_(0), range_max_(0) {}
        virtual ~Range() = default;

        T getLimitMin() const { return limit_min_; }
        T getLimitMax() const { return limit_max_; }
        T getRangeMin() const { return range_min_; }
        T getRangeMax() const { return range_max_; }
        T getLimit(RangeType rt) const
        {
            if (rt == RangeType::Min)
                return getLimitMin();
            return getLimitMax();
        }
        T getRange(RangeType rt) const
        {
            if (rt == RangeType::Min)
                return getRangeMin();
            return getRangeMax();
        }

        void resetRange(RangeType rt)
        {
            if (rt == RangeType::Min)
                range_min_ = limit_min_;
            else
                range_max_ = limit_max_;
        }

        void resetRanges()
        {
            resetRange(RangeType::Min);
            resetRange(RangeType::Max);
        }

        void setLimits(T limit_min, T limit_max)
        {
            limit_min_ = limit_min;
            limit_max_ = limit_max;

            //range is reset to new limits
            resetRanges();
        }

        void setRange(RangeType rt, T value)
        {
            if (rt == RangeType::Min)
                range_min_ = value;
            else
                range_max_ = value;
        }
        
        void setRange(T range_min, T range_max)
        {
            setRange(RangeType::Min, range_min);
            setRange(RangeType::Max, range_max);
        }

        bool isValid() const
        {
            return (limit_min_ < limit_max_ && 
                    range_min_ <= range_max_);
        }
        
        /**
         * Default handles range types which can be converted to double.
         * 
         * SPECIALIZE FOR TYPE SPECIFIC BEHAVIOR
         */
        double interpFactor(RangeType rt) const
        {
            if (!isValid())
                return 0.0;

            const T lrange = limit_max_ - limit_min_;

            //check if range is very small
            if (lrange < 1e-09)
                return 0.0;

            const T      v         = getRange(rt);
            const T      v_clamped = std::max(limit_min_, std::min(limit_max_, v));
            const double f         = (double)(v_clamped - limit_min_) / (double)lrange;

            return f;
        }

        /**
         * Default handles range types which can be converted to double.
         * factor in [0, 1].
         * 
         * SPECIALIZE FOR TYPE SPECIFIC BEHAVIOR
         */
        void interpolate(RangeType rt, double factor)
        {
            if (!isValid())
                return;

            const double f2 = std::max(0.0, std::min(1.0, factor));
            T v = T((1.0 - f2) * limit_min_ + f2 * limit_max_);

            if (rt == RangeType::Min)
                range_min_ = v;
            else
                range_max_ = v;
        }

        void print() const
        {
            std::cout << "[" << limit_min_ << "," << limit_max_ << "] range min: " << range_min_ << ", range max: " << range_max_ << std::endl;
        }

    protected:
        T limit_min_;
        T limit_max_;
        T range_min_;
        T range_max_;
    };

    /**
     * Represents a 'serializable' range, meaning a range whose values and limits can be set and retrieved as strings.
     */
    template<typename T>
    class SerializableRange : public Range<T>
    {
    public:
        SerializableRange() = default;
        virtual ~SerializableRange() = default;

        void setPrecision(int p) { precision_ = p; }

        QString getRangeMinAsString() const 
        { 
            return stringFromValue(this->range_min_, precision_); 
        }
        QString getRangeMaxAsString() const 
        { 
            return stringFromValue(this->range_max_, precision_); 
        }
        QString getRangeAsString(RangeType rt) const 
        { 
            return stringFromValue(this->getRange(rt), precision_); 
        }

        bool setLimits(const QString& minimum, const QString& maximum)
        {
            try
            {
                Range<T>::setLimits(stringToValue(minimum), 
                                    stringToValue(maximum));
            }
            catch(const std::exception& e)
            {
                return false;
            }
            return Range<T>::isValid();
        }

        bool setRange(RangeType rt, const QString& value)
        {
            try
            {
                Range<T>::setRange(rt, stringToValue(value));
            }
            catch(const std::exception& e)
            {
                return false;
            }
            return Range<T>::isValid(); 
        }

        bool setRange(const QString& range_min, const QString& range_max)
        {
            try
            {
                Range<T>::setRange(stringToValue(range_min), 
                                   stringToValue(range_max));
            }
            catch(const std::exception& e)
            {
                return false;
            }
            return Range<T>::isValid(); 
        }

    private:
        inline T stringToValue(const QString& s) const { return T(0); }
        inline QString stringFromValue(T value, int precision) const { return QString(); }

        int precision_ = 6;
    };

    //ADD STRING CONVERSION FUNCTIONALITY HERE
    template<>
    inline double SerializableRange<double>::stringToValue(const QString& s) const 
    { 
        bool ok;
        const double value = s.toDouble(&ok);

        if (!ok)
            throw std::runtime_error("SerializableRange::stringToValue: conversion failed");

        return value;
    }
    template<>
    inline float SerializableRange<float>::stringToValue(const QString& s) const 
    { 
        bool ok;
        const float value = s.toFloat(&ok);

        if (!ok)
            throw std::runtime_error("SerializableRange::stringToValue: conversion failed");

        return value;
    }
    template<>
    inline QString SerializableRange<double>::stringFromValue(double value, int precision) const 
    { 
        return QString::number(value, 'f', precision);
    }
    template<>
    inline QString SerializableRange<float>::stringFromValue(float value, int precision) const 
    { 
        return QString::number(value, 'f', precision);
    }
}

/**
 * Represents a range edit consisting of a two-fold slider.
 * Type specific behavior is added in child class.
 */
class RangeEditBase : public QWidget
{
    Q_OBJECT
public:
    typedef rangeedit::RangeType RangeType;

    RangeEditBase(int slider_steps = 1000, 
                  QWidget* parent = nullptr);
    virtual ~RangeEditBase() = default;

    void setLimits(const QString& v0, const QString& v1);
    void connectToFields(QLineEdit* min_field, 
                         QLineEdit* max_field);
signals:
    void lowerRangeChanged(QString);
    void upperRangeChanged(QString);
    void rangeChanged(QString, QString);

public slots:
    void setLowerRange(const QString& v);
    void setUpperRange(const QString& v);

protected:
    int getSliderValue(RangeType limit) const;
    double sliderFactor(RangeType limit) const;

    void setErrorState(bool error);

    void updateSlider();
    void updateLowerRange();
    void updateUpperRange();

    virtual bool setLimits_impl(const QString& v0, const QString& v1) = 0;
    virtual bool setLowerRange_impl(const QString& v) = 0;
    virtual bool setUpperRange_impl(const QString& v) = 0;

    virtual void updateSlider(int& n0, int& n1) = 0;
    virtual void updateRange(QString& s0, QString& s1, RangeType limit) = 0;

    static const int MinSliderSteps = 10;

    RangeSlider* slider_ = nullptr;

    bool range_error_ = false;
};

/**
 * Range edit encapsulating various range types.
 */
template <typename T>
class RangeEditT : public RangeEditBase 
{
public:
    RangeEditT(int slider_steps = 1000, 
               int precision = 6,
               QWidget* parent = nullptr);
    virtual ~RangeEditT() = default;

protected:
    bool setLimits_impl(const QString& v0, const QString& v1) override final;
    bool setLowerRange_impl(const QString& v) override final;
    bool setUpperRange_impl(const QString& v) override final;

    void updateSlider(int& n0, int& n1) override final;
    void updateRange(QString& s0, QString& s1, RangeType limit) override final;

    rangeedit::SerializableRange<T> range_;
};

#define DECLARE_RANGE_EDIT(Type,ClassName)                         \
    class ClassName : public RangeEditT<Type>                      \
    {                                                              \
    public:                                                        \
        ClassName(int slider_steps = 1000,                         \
                  int precision = 6,                               \
                  QWidget* parent = nullptr)                       \
            : RangeEditT<Type>(slider_steps, precision, parent) {} \
        virtual ~ClassName() = default;                            \
    };

//CONVENIENCE
DECLARE_RANGE_EDIT(float,  RangeEditFloat)
DECLARE_RANGE_EDIT(double, RangeEditDouble)
