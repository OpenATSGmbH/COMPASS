#pragma once

#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QWheelEvent>

class NoScrollQSpinBox : public QSpinBox {
public:
    using QSpinBox::QSpinBox; // Inherit constructors

protected:
    void wheelEvent(QWheelEvent *event) override {
        // Do not call the base class QSpinBox::wheelEvent(event).
        // Ignore the event so it propagates to the parent widget (e.g. enabling page scrolling).
        event->ignore();
    }
};

class NoScrollQDoubleSpinBox : public QDoubleSpinBox {
public:
    using QDoubleSpinBox::QDoubleSpinBox; // Inherit constructors

protected:
    void wheelEvent(QWheelEvent *event) override {
        // Do not call the base class QSpinBox::wheelEvent(event).
        // Ignore the event so it propagates to the parent widget (e.g. enabling page scrolling).
        event->ignore();
    }
};