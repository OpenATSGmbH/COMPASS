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

#include <QSlider>

namespace qxt
{

    template <typename PUB>
    class QxtPrivate
    {
    public:
        virtual ~QxtPrivate()
        {}
        inline void QXT_setPublic(PUB* pub)
        {
            qxt_p_ptr = pub;
        }

    protected:
        inline PUB& qxt_p()
        {
            return *qxt_p_ptr;
        }
        inline const PUB& qxt_p() const
        {
            return *qxt_p_ptr;
        }
        inline PUB* qxt_ptr()
        {
            return qxt_p_ptr;
        }
        inline const PUB* qxt_ptr() const
        {
            return qxt_p_ptr;
        }

    private:
        PUB* qxt_p_ptr;
    };

    template <typename PUB, typename PVT>
    class QxtPrivateInterface
    {
        friend class QxtPrivate<PUB>;
    public:
        QxtPrivateInterface()
        {
            pvt = new PVT;
        }
        ~QxtPrivateInterface()
        {
            delete pvt;
        }

        inline void setPublic(PUB* pub)
        {
            pvt->QXT_setPublic(pub);
        }
        inline PVT& operator()()
        {
            return *static_cast<PVT*>(pvt);
        }
        inline const PVT& operator()() const
        {
            return *static_cast<PVT*>(pvt);
        }
        inline PVT * operator->()
        {
        return static_cast<PVT*>(pvt);
        }
        inline const PVT * operator->() const
        {
        return static_cast<PVT*>(pvt);
        }
    private:
        QxtPrivateInterface(const QxtPrivateInterface&) { }
        QxtPrivateInterface& operator=(const QxtPrivateInterface&) { }
        QxtPrivate<PUB>* pvt;
    };

}

#define QXT_DECLARE_PRIVATE(PUB) friend class PUB##Private; qxt::QxtPrivateInterface<PUB, PUB##Private> qxt_d;
#define QXT_DECLARE_PUBLIC(PUB) friend class PUB;
#define QXT_INIT_PRIVATE(PUB) qxt_d.setPublic(this);
#define QXT_D(PUB) PUB##Private& d = qxt_d()
#define QXT_P(PUB) PUB& p = qxt_p()

class RangeSliderPrivate;

/**
 */
class RangeSlider : public QSlider
{
    Q_OBJECT
    QXT_DECLARE_PRIVATE(RangeSlider)
    Q_PROPERTY(int lowerValue READ lowerValue WRITE setLowerValue)
    Q_PROPERTY(int upperValue READ upperValue WRITE setUpperValue)
    Q_PROPERTY(int lowerPosition READ lowerPosition WRITE setLowerPosition)
    Q_PROPERTY(int upperPosition READ upperPosition WRITE setUpperPosition)
    Q_PROPERTY(HandleMovementMode handleMovementMode READ handleMovementMode WRITE setHandleMovementMode)
    Q_ENUMS(HandleMovementMode)

public:
    explicit RangeSlider(QWidget* parent = nullptr);
    explicit RangeSlider(Qt::Orientation orientation, QWidget* parent = nullptr);
    virtual ~RangeSlider();

    enum HandleMovementMode
    {
        FreeMovement,
        NoCrossing,
        NoOverlapping
    };

    enum SpanHandle
    {
        NoHandle,
        LowerHandle,
        UpperHandle
    };

    HandleMovementMode handleMovementMode() const;
    void setHandleMovementMode(HandleMovementMode mode);

    int lowerValue() const;
    int upperValue() const;

    int lowerPosition() const;
    int upperPosition() const;

public Q_SLOTS:
    void setLowerValue(int lower);
    void setUpperValue(int upper);
    void setSpan(int lower, int upper);

    void setLowerPosition(int lower);
    void setUpperPosition(int upper);

Q_SIGNALS:
    void spanChanged(int lower, int upper);
    void lowerValueChanged(int lower);
    void upperValueChanged(int upper);

    void lowerPositionChanged(int lower);
    void upperPositionChanged(int upper);

    void sliderPressed(SpanHandle handle);

protected:
    virtual void keyPressEvent(QKeyEvent* event);
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void paintEvent(QPaintEvent* event);
};
