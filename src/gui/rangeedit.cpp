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

#include "rangeedit.h"
#include "rangeslider.h"

#include <QHBoxLayout>
#include <QLineEdit>

/****************************************************************************************
 * RangeEditBase
 ****************************************************************************************/

/**
 */
RangeEditBase::RangeEditBase(int slider_steps, QWidget* parent)
:   QWidget(parent)
{
    QHBoxLayout* l = new QHBoxLayout;
    setLayout(l);

    //create slider controls
    slider_ = new RangeSlider;
    l->addWidget(slider_);

    slider_->setHandleMovementMode(RangeSlider::HandleMovementMode::NoCrossing);
    slider_->setOrientation(Qt::Orientation::Horizontal);
    slider_->setMinimum(0);
    slider_->setMaximum(std::max(MinSliderSteps, slider_steps));
    slider_->setLowerValue(slider_->minimum());
    slider_->setUpperValue(slider_->maximum());

    connect(slider_, &RangeSlider::lowerValueChanged, this, &RangeEditBase::updateLowerRange);
    connect(slider_, &RangeSlider::upperValueChanged, this, &RangeEditBase::updateUpperRange);
}

/**
 */
void RangeEditBase::setLimits(const QString& v0, const QString& v1)
{
    bool ok = setLimits_impl(v0, v1);

    if (ok)
        updateSlider();

    setErrorState(!ok);
}

/**
 */
void RangeEditBase::setLowerRange(const QString& v)
{
    bool ok = setLowerRange_impl(v);

    if (ok)
        updateSlider();

    setErrorState(!ok);
}

/**
 */
void RangeEditBase::setUpperRange(const QString& v)
{
    bool ok = setUpperRange_impl(v);

    if (ok)
        updateSlider();

    setErrorState(!ok);
}

/**
 */
int RangeEditBase::getSliderValue(RangeType limit) const
{
    return limit == RangeType::Min ? slider_->lowerValue() : slider_->upperValue();
}

/**
 */
double RangeEditBase::sliderFactor(RangeType limit) const
{
    return (double)(getSliderValue(limit) - slider_->minimum()) / (double)(slider_->maximum() - slider_->minimum());
}

/**
 */
void RangeEditBase::updateLowerRange()
{
    setErrorState(false);

    QString s0, s1;
    updateRange(s0, s1, RangeType::Min);

    emit lowerRangeChanged(s0);
    emit rangeChanged(s0, s1);
}

/**
 */
void RangeEditBase::updateUpperRange()
{
    setErrorState(false);

    QString s0, s1;
    updateRange(s0, s1, RangeType::Max);

    emit upperRangeChanged(s1);
    emit rangeChanged(s0, s1);
}

/**
 */
void RangeEditBase::updateSlider()
{
    //get new slider values
    int n0, n1;
    updateSlider(n0, n1);

    //update slider
    disconnect(slider_, &RangeSlider::lowerValueChanged, this, &RangeEditBase::updateLowerRange);
    disconnect(slider_, &RangeSlider::upperValueChanged, this, &RangeEditBase::updateUpperRange);

    slider_->setLowerValue(n0);
    slider_->setUpperValue(n1);

    connect(slider_, &RangeSlider::lowerValueChanged, this, &RangeEditBase::updateLowerRange);
    connect(slider_, &RangeSlider::upperValueChanged, this, &RangeEditBase::updateUpperRange);
}

/**
 */
void RangeEditBase::connectToFields(QLineEdit* min_field, 
                                    QLineEdit* max_field)
{
    if (!min_field || !max_field)
        return;

    auto minChangedCB = [=] (const QString& str)
    {
        //std::cout << "MIN changed to " << str.toStdString() << std::endl;
        min_field->setText(str);
        min_field->textEdited(min_field->text());
    };
    auto maxChangedCB = [=] (const QString& str)
    {
        //std::cout << "MAX changed to " << str.toStdString() << std::endl;
        max_field->setText(str);
        max_field->textEdited(max_field->text());
    };
    auto adjustMinCB = [=] ()
    {
        this->setLowerRange(min_field->text());
    };
    auto adjustMaxCB = [=] ()
    {
        this->setUpperRange(max_field->text());
    };

    adjustMinCB();
    adjustMaxCB();

    connect(this, &RangeEditBase::lowerRangeChanged, minChangedCB);
    connect(this, &RangeEditBase::upperRangeChanged, maxChangedCB);
    connect(min_field, &QLineEdit::textChanged, adjustMinCB);
    connect(max_field, &QLineEdit::textChanged, adjustMaxCB);
}

/**
 */
void RangeEditBase::setErrorState(bool error)
{
    range_error_ = error;
    setEnabled(!error);
}

/****************************************************************************************
 * RangeEditT
 ****************************************************************************************/

template<typename T>
RangeEditT<T>::RangeEditT(int slider_steps, 
                          int precision,
                          QWidget* parent)
:   RangeEditBase(slider_steps, parent)
{
    range_.setPrecision(precision);
}

template<typename T>
bool RangeEditT<T>::setLimits_impl(const QString& v0, const QString& v1)
{
    return range_.setLimits(v0, v1);
}

template<typename T>
bool RangeEditT<T>::setLowerRange_impl(const QString& v)
{
    return range_.setRange(RangeType::Min, v);
}

template<typename T>
bool RangeEditT<T>::setUpperRange_impl(const QString& v)
{
    return range_.setRange(RangeType::Max, v);
}

template<typename T>
void RangeEditT<T>::updateSlider(int& n0, int& n1)
{
    //get factors for range
    const double t0 = range_.interpFactor(RangeType::Min);
    const double t1 = range_.interpFactor(RangeType::Max);

    //get suitable slider ticks
    n0 = t0 * slider_->maximum();
    n1 = t1 * slider_->maximum();
}

template<typename T>
void RangeEditT<T>::updateRange(QString& s0, QString& s1, RangeType limit)
{
    //get new slider factor
    const double t = sliderFactor(limit);

    //interpolate range between limits
    range_.interpolate(limit, t);

    //get range as string
    s0 = range_.getRangeMinAsString();
    s1 = range_.getRangeMaxAsString();
}

// ADD EXPLICIT TEMPLATE INSTANTIATIONS HERE
template class RangeEditT<float>;
template class RangeEditT<double>;
