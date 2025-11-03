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


#include "labelplacement.h"
#include "labelplacement_force.h"
#include "labelplacement_spring.h"

#include <limits>
#include <iostream>

#include <Eigen/Core>

#include <QPainter>
#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QComboBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QTimer>
#include <QElapsedTimer>

/**
 */
LabelPlacementEngine::LabelPlacementEngine() = default;

/**
 * Add a new label.
 */
void LabelPlacementEngine::addLabel(const std::string& id, 
                                    double x_anchor, 
                                    double y_anchor, 
                                    double w,
                                    double h,
                                    double* x_init, 
                                    double* y_init,
                                    double* x_ref,
                                    double* y_ref)
{
    Label l;
    l.id       = id;
    l.x_anchor = x_anchor;
    l.y_anchor = y_anchor;
    l.x        = x_init ? *x_init : x_anchor;
    l.y        = y_init ? *y_init : y_anchor;
    l.x_last   = l.x;
    l.y_last   = l.y;
    l.w        = w;
    l.h        = h;

    if (x_ref && y_ref)
    {
        l.x_ref = *x_ref;
        l.y_ref = *y_ref;
    }

    auto it = label_map_.find(id);

    //label already registered -> overwrite
    if (it != label_map_.end())
    {
        labels_[ it->second ] = l;
        return;
    }

    //register new label
    size_t idx = labels_.size();

    labels_.push_back(l);
    label_map_[ id ] = idx;
}

/**
 * Obtain label by index.
 */
const LabelPlacementEngine::Label& LabelPlacementEngine::getLabel(size_t idx) const
{
    return labels_.at(idx);
}


/**
 * Obtain label by id.
 */
const LabelPlacementEngine::Label* LabelPlacementEngine::getLabel(const std::string& id) const
{
    auto it = label_map_.find(id);
    if (it == label_map_.end())
        return nullptr;

    return &labels_[ it->second ];
}

/**
 */
void LabelPlacementEngine::removeLabel(size_t idx)
{
    const auto& l = labels_.at(idx);

    label_map_.erase(l.id);
    labels_.erase(labels_.begin() + idx);
}

/**
 */
void LabelPlacementEngine::removeLabel(const std::string& id)
{
    auto it = label_map_.find(id);
    if (it == label_map_.end())
        return;
  
    labels_.erase(labels_.begin() + it->second);
    label_map_.erase(id);
}

/**
 */
void LabelPlacementEngine::clearLabels()
{
    labels_.clear();
    label_map_.clear();
}

/**
 * Reverts all label positions to initial positions.
 */
void LabelPlacementEngine::revertPlacements()
{
    for (auto& l : labels_)
    {
        l.revert();
    }    
}

/**
 * Main method for running automated label placement.
 */
bool LabelPlacementEngine::placeLabels()
{
    bool ok = false;
    if (settings_.method == Method::ForceBasedSimple ||
        settings_.method == Method::ForceBasedExtended)
        ok = label_placement::force::placeLabels(labels_, settings_);
    else if (settings_.method == Method::ForceBasedExact)
        ok = label_placement::force_exact::placeLabels(labels_, settings_);
    else
        return false; //unknown method

    //if something went wrong revert positions to initial ones
    if (!ok)
    {
        revertPlacements();
        return false;
    }

    return true;
}

/**
 */
void LabelPlacementEngine::showData(const TestConfig& test_config) const
{
    size_t n = labels_.size();

    if (n < 2)
        return;

    std::vector<TestLabel> test_labels(n);

    for (size_t i = 0; i < n; ++i)
    {
        const auto& l  = labels_[ i ];
        auto&       tl = test_labels[ i ];

        tl.label  = l;
        tl.x_init = l.x_last;
        tl.y_init = l.y_last;
        tl.txt    = l.id;
    }

    auto config = test_config;
    
    if (settings_.fb_avoid_anchors && settings_.fb_anchor_radius > 0)
    {
        config.radius        = settings_.fb_anchor_radius;
        config.avoid_anchors = true;
    }
    if (settings_.fb_avoid_roi && !settings_.roi.isEmpty())
    {
        config.roi       = settings_.roi;
        config.avoid_roi = true;
    }

    //convert coords to test window size in pixels
    computeScreenTransform(config, test_labels);

    //create test dialog
    QDialog dlg;

    QVBoxLayout* layoutv = new QVBoxLayout;
    dlg.setLayout(layoutv);

    QLabel* label = new QLabel;
    label->setFixedSize(config.width, config.height);

    QPushButton* closeButton = new QPushButton("Close");

    layoutv->addWidget(label);
    layoutv->addWidget(closeButton);

    QObject::connect(closeButton, &QPushButton::pressed, &dlg, &QDialog::accept);

    QImage img(config.width, config.height, QImage::Format_RGB32);
    img.fill(Qt::white);

    renderTestFrame(img, test_labels, config);
        
    label->setPixmap(QPixmap::fromImage(img));

    dlg.setWindowModality(Qt::WindowModality::ApplicationModal);
    dlg.exec();
}

/**
 */
void LabelPlacementEngine::runTest(const TestConfig& test_config) const
{
    std::vector<Label> labels;

    int n = test_config.num_objects;

    std::vector<TestLabel> test_labels(n);

    double range = 1000.0;

    auto config = test_config;
    config.speed      *= range;
    config.radius     *= range;
    config.label_w    *= range;
    config.label_h    *= range;
    config.label_offs *= range;

    //init random labels/positions
    for (int i = 0; i < n; ++i)
    {
        auto& l = test_labels[ i ];

        Eigen::Vector2d pos, dir;
        pos.setRandom();
        dir.setRandom();
        dir.normalize();

        pos += Eigen::Vector2d(1, 1);
        pos *= 0.5;
        pos *= range;

        Eigen::Vector2d v;
        v.setRandom();

        Eigen::Vector3d col;
        col.setRandom();
        col += Eigen::Vector3d(1, 1, 1);
        col *= 0.5;

        double speed = 0.9 + 0.1 * v.x();

        l.label.x_anchor = pos.x();
        l.label.y_anchor = pos.y();
        l.label.x        = l.label.x_anchor + config.label_offs;
        l.label.y        = l.label.y_anchor - config.label_offs;
        l.label.x_last   = l.label.x;
        l.label.y_last   = l.label.y;
        
        l.label.w        = config.label_w;
        l.label.h        = config.label_h;

        l.x_init         = l.label.x;
        l.y_init         = l.label.y;
        
        l.dirx           = dir.x();
        l.diry           = dir.y();
        l.speed          = speed;
        l.txt            = "Test" + std::to_string(i); //unique id is important
        l.color          = QColor(col.x() * 255, col.y() * 255, col.z() * 255);
    }

    config.roi = QRectF(0, 0, range, range);

    //convert coords to test window size in pixels
    computeScreenTransform(config, test_labels);

    //show test data in dialog
    runTest(test_labels, config);
}

/**
 */
void LabelPlacementEngine::runTest(const std::vector<TestLabel>& test_labels,
                                   const TestConfig& test_config) const
{
    auto config = test_config;

    std::vector<TestLabel> labels = test_labels;

    //create test dialog
    QDialog dlg;

    QHBoxLayout* layouth = new QHBoxLayout;
    dlg.setLayout(layouth);

    QLabel* label = new QLabel;
    label->setFixedSize(config.width, config.height);

    bool runs = false;

    QVBoxLayout* layoutv = new QVBoxLayout;

    layouth->addWidget(label);
    layouth->addLayout(layoutv);

    QPushButton* runButton = new QPushButton("Run");
    QPushButton* closeButton = new QPushButton("Close");

    QCheckBox* avoidAnchorsBox = new QCheckBox("Avoid anchors");
    QCheckBox* avoidROIBox = new QCheckBox("Avoid ROI");

    avoidAnchorsBox->setChecked(true);
    avoidROIBox->setChecked(true);

    QDoubleSpinBox* radiusBox = new QDoubleSpinBox;
    radiusBox->setDecimals(6);
    radiusBox->setMinimum(0);
    radiusBox->setMaximum(10000000);
    radiusBox->setValue(config.radius);

    QComboBox* autoCombo = new QComboBox;
    autoCombo->addItem("None", QVariant(-1));
    autoCombo->addItem("ForceSimple", QVariant((int)LabelPlacementEngine::Method::ForceBasedSimple));
    autoCombo->addItem("ForceExtended", QVariant((int)LabelPlacementEngine::Method::ForceBasedExtended));
    autoCombo->addItem("ForceExact", QVariant((int)LabelPlacementEngine::Method::ForceBasedExact));

    QComboBox* forceDirCombo = new QComboBox;
    forceDirCombo->addItem("X" , QVariant((int)label_placement::ForceDirection::X ));
    forceDirCombo->addItem("Y" , QVariant((int)label_placement::ForceDirection::Y ));
    forceDirCombo->addItem("XY", QVariant((int)label_placement::ForceDirection::XY));
    forceDirCombo->setCurrentIndex(2);

    layoutv->addWidget(autoCombo);
    layoutv->addWidget(forceDirCombo);
    layoutv->addWidget(avoidAnchorsBox);
    layoutv->addWidget(avoidROIBox);
    layoutv->addWidget(radiusBox);

    layoutv->addSpacerItem(new QSpacerItem(5, 5, QSizePolicy::Fixed, QSizePolicy::Expanding));

    layoutv->addWidget(runButton);
    layoutv->addWidget(closeButton);

    QObject::connect(closeButton, &QPushButton::pressed, &dlg, &QDialog::accept);

    //callback for the run button
    auto runCB = [&] () 
    {
        if (runs)
            runButton->setText("Run");
        else
            runButton->setText("Pause");

        runs = !runs;
    };

    QObject::connect(runButton, &QPushButton::pressed, runCB);

    int n = (int)test_labels.size();

    //updates the display every time the timer elapses
    auto updateCB = [ & ] () 
    {
        if (!runs || n < 1)
            return;

        config.radius        = radiusBox->value();
        config.avoid_roi     = avoidROIBox->isChecked();
        config.avoid_anchors = avoidAnchorsBox->isChecked();

        const int w = test_config.width;
        const int h = test_config.height;

        //init display image
        QImage img(w, h, QImage::Format_RGB32);
        img.fill(Qt::white);

        //update position and init labels
        for (int i = 0; i < n; ++i)
        {
            auto& l = labels[ i ];

            //this is in label space
            l.label.x_anchor += l.dirx * config.speed * l.speed;
            l.label.y_anchor += l.diry * config.speed * l.speed;

            if (config.avoid_roi && !config.roi.isEmpty())
            {
                if (l.label.x_anchor < config.roi.left()  || 
                    l.label.y_anchor < config.roi.top()   || 
                    l.label.x_anchor > config.roi.right() || 
                    l.label.y_anchor > config.roi.bottom())
                {
                    l.dirx *= -1;
                    l.diry *= -1;
                }
            }

            l.label.x = l.label.x_anchor + config.label_offs;
            l.label.y = l.label.y_anchor - config.label_offs;
            l.x_init  = l.label.x;
            l.y_init  = l.label.y;
        }

        int mode = autoCombo->currentData().toInt();

        //place labels automatically if desired
        if (mode >= 0)
        {
            QElapsedTimer t;
            t.start();

            LabelPlacementEngine::Method    method    = (LabelPlacementEngine::Method)mode;
            label_placement::ForceDirection force_dir = (label_placement::ForceDirection)forceDirCombo->currentData().toInt();

            LabelPlacementEngine labelPlacement;
            auto& settings = labelPlacement.settings();
            settings.method           = method;
            settings.roi              = config.roi;

            settings.fb_avoid_anchors = config.avoid_anchors;
            settings.fb_anchor_radius = config.radius;
            settings.fb_avoid_roi     = config.avoid_roi;

            settings.fbe_anchor_radius = config.radius;
            settings.fbe_force_dir     = force_dir;

            //collect labels to place
            for (int i = 0; i < n; ++i)
            {
                auto& l = labels[ i ];
                labelPlacement.addLabel(l.txt, 
                                        l.label.x_anchor, 
                                        l.label.y_anchor, 
                                        l.label.w, 
                                        l.label.h, 
                                        &l.label.x, 
                                        &l.label.y);
            }

            //run placement
            labelPlacement.placeLabels();

            //retrieve optimized position
            for (int i = 0; i < n; ++i)
            {
                auto& l = labels[ i ];
                const auto& l_opt = labelPlacement.getLabel(i);
                l.label.x = l_opt.x;
                l.label.y = l_opt.y;
            }

            loginf << "auto placement of labels in " << t.elapsed() << "ms";
        }
        
        renderTestFrame(img, labels, config);
        
        //show new canva
        label->setPixmap(QPixmap::fromImage(img));

        loginf << "update " << label->width() << "x" << label->height();
    };

    //run update every 1s
    QTimer t;
    t.setInterval(config.interval_ms);
    t.setSingleShot(false);
    QObject::connect(&t, &QTimer::timeout, updateCB);
    t.start();

    dlg.exec();
}

/**
 */
void LabelPlacementEngine::renderTestFrame(QImage& img, 
                                           const std::vector<TestLabel>& labels,
                                           const TestConfig& test_config) const
{
    QPainter p(&img);

    auto convert = [ & ] (double& x, double& y, bool scale_only) 
    {
        if (test_config.screen_transform)
            test_config.screen_transform(x, y, scale_only);
    };
    auto convertBBox = [ & ] (double& x, double& y, double& w, double& h) 
    {
        if (test_config.screen_transform_bbox)
            test_config.screen_transform_bbox(x, y, w, h);
    };

    if (test_config.avoid_roi && !test_config.roi.isEmpty())
    {
        double x0 = test_config.roi.left();
        double y0 = test_config.roi.top();
        double x1 = test_config.roi.right();
        double y1 = test_config.roi.bottom();

        convert(x0, y0, false);
        convert(x1, y1, false);

        double xmin = std::min(x0, x1);
        double ymin = std::min(y0, y1);
        double xmax = std::max(x0, x1);
        double ymax = std::max(y0, y1);

        QPen pen;
        pen.setColor(Qt::blue);
        pen.setStyle(Qt::PenStyle::DotLine);

        p.save();
        p.setPen(pen);
        p.drawRect(xmin, ymin, xmax - xmin, ymax - ymin);
        p.restore();
    }

    for (const auto& l : labels)
    {
        double x  = l.label.x;
        double y  = l.label.y;
        double xa = l.label.x_anchor;
        double ya = l.label.y_anchor;
        double w  = l.label.w;
        double h  = l.label.h;

        double xi = l.x_init.has_value() ? l.x_init.value() : 0.0;
        double yi = l.y_init.has_value() ? l.y_init.value() : 0.0;
        
        double rx = test_config.radius;
        double ry = test_config.radius;

        //convert from data space to screen space
        convertBBox(x, y, w, h);

        convert(xa, ya, false);
        convert(xi, yi, false);
        convert(rx, ry, true);

        //draw markers
        p.save();
        p.setBrush(QBrush(l.color, Qt::BrushStyle::SolidPattern));
        p.drawEllipse(QPointF(xa, ya), rx, ry);
        p.restore();

        //draw connection lines
        p.save();
        p.drawLine(QPointF(xa, ya), QPointF(x, y));
        p.restore();

        //draw labels
        p.save();
        p.setBrush(QBrush(l.color, Qt::BrushStyle::SolidPattern));
        p.drawRect(x, y, w, h);
        p.restore();

        //draw label paths
        if (l.x_init.has_value() && l.y_init.has_value())
        {
            p.save();
            p.setBrush(QBrush(Qt::red, Qt::BrushStyle::SolidPattern));
            p.drawEllipse(QPointF(xi, yi), 2, 2);
            p.restore();

            p.save();
            QPen pen;
            pen.setColor(Qt::red);
            pen.setStyle(Qt::PenStyle::DotLine);
            p.setPen(pen);
            p.drawLine(QPointF(xi, yi), QPointF(x, y));
            p.restore();
        }
    }
}

/**
 */
void LabelPlacementEngine::computeScreenTransform(const TestConfig& test_config, 
                                                  const std::vector<TestLabel>& test_labels) const                                
{
    test_config.screen_transform      = TestConfig::ScreenTransform();
    test_config.screen_transform_bbox = TestConfig::ScreenTransformBBox();

    int n = (int)test_labels.size();
    if (n < 2)
        return;

    //determine bounds
    double xmin = std::numeric_limits<double>::max();
    double ymin = std::numeric_limits<double>::max();
    double xmax = std::numeric_limits<double>::min();
    double ymax = std::numeric_limits<double>::min();

    auto checkPos = [ & ] (double x, double y) 
    {
        if (x < xmin)
            xmin = x;
        if (y < ymin)
            ymin = y;
        if (x > xmax)
            xmax = x;
        if (y > ymax)
            ymax = y;
    };
 
    for (int i = 0; i < n; ++i)
    {
        const auto& l = test_labels[ i ];

        checkPos(l.label.x, l.label.y);
        checkPos(l.label.x_anchor, l.label.y_anchor);

        if (l.x_init.has_value() && l.y_init.has_value())
            checkPos(*l.x_init, *l.y_init);
    }

    if (!test_config.roi.isEmpty())
    {
        xmin = test_config.roi.left();
        ymin = test_config.roi.top();
        xmax = test_config.roi.right();
        ymax = test_config.roi.bottom();
    }

    const double ws = test_config.width;
    const double hs = test_config.height;

    const double w = xmax - xmin;
    const double h = ymax - ymin;

    //fit data window into test window size
    double scale = 1.0;

    const double inner_aspect_ratio = w  / h;
    const double outer_aspect_ratio = ws / hs;
    if (inner_aspect_ratio < outer_aspect_ratio) 
        scale = hs / h;
    else
        scale = ws / w;

    //center the fitted window
    double offsx = (ws - w * scale) * 0.5;
    double offsy = (hs - h * scale) * 0.5;

    const int h_win = test_config.height;

    bool flipy = test_config.flip_y;

    //store screen transform function to config
    auto screen_transform_func = [ = ] (double& x, double& y, bool scale_only) 
    {
        if (!scale_only)
        {
            x -= xmin;
            y -= ymin;
        }
        x *= scale;
        y *= scale;
        if (!scale_only)
        {
            x += offsx;
            y += offsy;

            if (flipy)
            {
                y = h_win - y;
            }
        }
    };
    auto screen_transform_bbox_func = [ = ] (double& x, double& y, double& w, double& h) 
    {
        x -= xmin;
        y -= ymin;   
        x *= scale;
        y *= scale;
        x += offsx;
        y += offsy;
        if (flipy)
        {
            y  = h_win - y;
            y -= h;
        }
        w *= scale;
        h *= scale;
    };

    test_config.screen_transform      = screen_transform_func;
    test_config.screen_transform_bbox = screen_transform_bbox_func;
}
