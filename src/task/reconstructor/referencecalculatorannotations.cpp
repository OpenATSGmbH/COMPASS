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

#include "referencecalculatorannotations.h"
#include "measurement.h"
#include "kalman_projection.h"
#include "viewpointgenerator.h"
#include "histograminitializer.h"
#include "kalman_estimator.h"
#include "reconstructorbase.h"
#include "dbcontentmanager.h"
#include "datasourcemanager.h"
#include "compass.h"

#include "number.h"
#include "logger.h"

namespace refcalc_annotations
{

/**
 */
ReferenceCalculatorAnnotations::ReferenceCalculatorAnnotations() = default;

/**
 */
ReferenceCalculatorAnnotations::~ReferenceCalculatorAnnotations() = default;

/**
*/
bool ReferenceCalculatorAnnotations::hasAnnotations() const
{
    return !annotation_data_.empty();
}

/**
*/
void ReferenceCalculatorAnnotations::setReconstructor(const ReconstructorBase* reconstructor)
{
    reconstructor_ = reconstructor;
}

/**
*/
TRAnnotation ReferenceCalculatorAnnotations::createTRAnnotation(const reconstruction::Measurement& mm,
                                                                const boost::optional<Eigen::Vector2d>& speed_pos,
                                                                const boost::optional<Eigen::Vector2d>& accel_pos,
                                                                const boost::optional<Eigen::Vector2d>& input_pos,
                                                                const boost::optional<std::string>& info_str) const
{
    TRAnnotation a;
    a.ts            = mm.t;
    a.pos_wgs84     = Eigen::Vector2d(mm.lat, mm.lon);
    a.speed_pos     = speed_pos;
    a.acc_pos       = accel_pos;
    a.pos_mm        = input_pos;
    a.pos_mm_uncorr = input_pos;
    a.Q_var         = mm.Q_var;
    a.info          = info_str;

    if (reconstructor_ && mm.source_id.has_value())
    {
        auto tr_info = reconstructor_->getInfo(mm.source_id.value());

        if (tr_info)
        {
            a.pos_mm        = mm.t == tr_info->timestamp_ ? Eigen::Vector2d(tr_info->position()->latitude_, tr_info->position()->longitude_) : input_pos;
            a.pos_mm_uncorr = mm.t == tr_info->timestamp_ ? Eigen::Vector2d(tr_info->position_->latitude_, tr_info->position_->longitude_)   : input_pos;
        }
    }

    if (mm.hasStdDevPosition())
        a.accuracy = Eigen::Vector3d(mm.x_stddev.value(), mm.y_stddev.value(), mm.xy_cov.has_value() ? mm.xy_cov.value() : 0.0);
    if (mm.hasVelocity())
        a.speed = std::sqrt(mm.vx.value() * mm.vx.value() + mm.vy.value() * mm.vy.value());
    if (mm.hasAcceleration())
        a.accel = std::sqrt(mm.ax.value() * mm.ax.value() + mm.ay.value() * mm.ay.value());

    return a;
}

/**
*/
TRAnnotation ReferenceCalculatorAnnotations::createTRAnnotation(const reconstruction::Reference& ref,
                                                                const boost::optional<Eigen::Vector2d>& speed_pos,
                                                                const boost::optional<Eigen::Vector2d>& accel_pos,
                                                                const boost::optional<Eigen::Vector2d>& input_pos) const
{
    const reconstruction::Measurement* mm = &ref;
    auto anno = createTRAnnotation(*mm, speed_pos, accel_pos, input_pos, {});

    anno.reset       = ref.reset_pos;
    anno.proj_change = ref.projchange_pos;

    return anno;
}

/**
*/
TRAnnotation ReferenceCalculatorAnnotations::createTRAnnotation(const kalman::KalmanUpdate& update,
                                                                const reconstruction::KalmanEstimator& estimator,
                                                                reconstruction::KalmanProjectionHandler& phandler,
                                                                const Eigen::Vector2d& proj_center,
                                                                int submodel_idx,
                                                                bool debug,
                                                                const std::string& name) const
{
    reconstruction::Measurement mm;
    boost::optional<Eigen::Vector2d> speed_pos;
    boost::optional<Eigen::Vector2d> accel_pos;

    estimator.storeUpdateAndUnproject(mm, update, phandler, &speed_pos, &accel_pos, submodel_idx);

    if (debug)
        loginf << name << ": pos " << mm.lat << " " << mm.lon;

    return createTRAnnotation(mm, speed_pos, accel_pos, {}, {});
}

/**
*/
RTSAnnotation ReferenceCalculatorAnnotations::createRTSAnnotation(const kalman::RTSDebugInfo& rts_debug_info,
                                                                  const reconstruction::KalmanEstimator& estimator,
                                                                  reconstruction::KalmanProjectionHandler& phandler) const
{
    RTSAnnotation anno;

    traced_assert(rts_debug_info.state0.imm_state);

    size_t nm = rts_debug_info.state0.imm_state->filter_states.size();

    anno.rts_step_models.resize(nm);
    anno.rts_step.color = QColor(255, 255, 255);

    auto createUpdate = [ & ] (const kalman::KalmanState& state)
    {
        kalman::KalmanUpdate update;
        update.projection_center = rts_debug_info.projection_center;
        update.state             = state;

        return update;
    };

    kalman::KalmanUpdate update_state0        = createUpdate(rts_debug_info.state0);
    kalman::KalmanUpdate update_state0_smooth = createUpdate(rts_debug_info.state0_smooth);
    kalman::KalmanUpdate update_state1        = createUpdate(rts_debug_info.state1);
    kalman::KalmanUpdate update_state1_smooth = createUpdate(rts_debug_info.state1_smooth);

    bool debug = false;

    anno.rts_step.state0        = createTRAnnotation(update_state0       , estimator, phandler, rts_debug_info.projection_center, -1, debug, "state0");
    anno.rts_step.state1        = createTRAnnotation(update_state1       , estimator, phandler, rts_debug_info.projection_center, -1, debug, "state1");
    anno.rts_step.state0_smooth = createTRAnnotation(update_state0_smooth, estimator, phandler, rts_debug_info.projection_center, -1, debug, "state0_smooth");
    anno.rts_step.state1_smooth = createTRAnnotation(update_state1_smooth, estimator, phandler, rts_debug_info.projection_center, -1, debug, "state1_smooth");

    for (size_t i = 0; i < nm; ++i)
    {
        auto& anno_model = anno.rts_step_models.at(i);

        std::string name = "model" + std::to_string(i);
        
        anno_model.color         = QColor(i % 3 == 0 ? 255 : 0, i % 3 == 1 ? 255 : 0, i % 3 == 2 ? 255 : 0);
        anno_model.state0        = createTRAnnotation(update_state0       , estimator, phandler, rts_debug_info.projection_center, (int)i, debug, name + "state0");
        anno_model.state1        = createTRAnnotation(update_state1       , estimator, phandler, rts_debug_info.projection_center, (int)i, debug, name + "state1");
        anno_model.state0_smooth = createTRAnnotation(update_state0_smooth, estimator, phandler, rts_debug_info.projection_center, (int)i, debug, name + "state0_smooth");
        anno_model.state1_smooth = createTRAnnotation(update_state1_smooth, estimator, phandler, rts_debug_info.projection_center, (int)i, debug, name + "state1_smooth");
    }

    std::stringstream ss;
    for (int i = 0; i < rts_debug_info.mu.size(); ++i)
        ss << rts_debug_info.mu[ i ] << " ";

    anno.extra_info = "";
    anno.extra_info += ss.str();

    return anno;
}

/**
*/
IMMAnnotation ReferenceCalculatorAnnotations::createIMMAnnotation(const kalman::KalmanUpdate& imm_update,
                                                                  const reconstruction::KalmanEstimator& estimator,
                                                                  reconstruction::KalmanProjectionHandler& phandler) const
{
    traced_assert(imm_update.state.imm_state);

    size_t nm = imm_update.state.imm_state->filter_states.size();

    IMMAnnotation anno;

    anno.imm_step_models.resize(nm);
    anno.imm_step.state = createTRAnnotation(imm_update, estimator, phandler, imm_update.projection_center, -1, false, "");
    anno.imm_step.color = QColor(255, 255, 255);

    for (size_t i = 0; i < nm; ++i)
    {
        auto& anno_model = anno.imm_step_models.at(i);

        std::string name = "model" + std::to_string(i);
        
        anno_model.color = QColor(i % 3 == 0 ? 255 : 0, i % 3 == 1 ? 255 : 0, i % 3 == 2 ? 255 : 0);
        anno_model.state = createTRAnnotation(imm_update, estimator, phandler, imm_update.projection_center, (int)i, false, "");
    }

    std::stringstream ss;
    for (int i = 0; i < imm_update.state.imm_state->mu.size(); ++i)
        ss << imm_update.state.imm_state->mu[ i ] << " ";

    anno.extra_info = "";
    anno.extra_info += ss.str();

    return anno;
}

/**
*/
void ReferenceCalculatorAnnotations::addAnnotationData(const reconstruction::KalmanEstimator& estimator, 
                                                       const std::string& name,
                                                       const AnnotationStyle& style,
                                                       const boost::optional<AnnotationStyle>& style_osg,
                                                       const std::vector<kalman::KalmanUpdate>& updates,
                                                       bool debug_imm,
                                                       const std::map<Key, ReferenceCalculatorInputInfo>* input_infos,
                                                       const std::vector<QPointF>* fail_pos,
                                                       const std::vector<QPointF>* skip_pos,
                                                       const std::map<Key, kalman::RTSDebugInfo>* rts_debug_infos,
                                                       bool debug)
{
    //store updates to references for easier access
    std::vector<reconstruction::Reference> references;
    std::vector<boost::optional<Eigen::Vector2d>> speed_positions, accel_positions;
    estimator.storeUpdates(references, updates, &speed_positions, &accel_positions);

    std::vector<TRAnnotation> annos;
    annos.reserve(references.size());

    const auto* slice = reconstructor_ ? &reconstructor_->currentSlice() : nullptr;

    for (size_t i = 0; i < references.size(); ++i)
    {
        const auto& r = references[ i ];

        if (slice && !slice->isWritten(r.t)) 
            continue;

        //try to obtain corresponding input measurement's position
        boost::optional<Eigen::Vector2d> input_pos;
        if (input_infos && r.source_id.has_value())
        {
            auto it = input_infos->find(r.uniqueID());
            if (it != input_infos->end())
                input_pos = Eigen::Vector2d(it->second.lat, it->second.lon);
        }

        auto a = createTRAnnotation(r, 
                                    speed_positions[ i ], 
                                    accel_positions[ i ],
                                    input_pos);

        annos.push_back(a);
    }

    reconstruction::KalmanProjectionHandler phandler;

    std::map<Key, IMMAnnotation> imm_annotations;
    if (debug_imm)
    {
        for (const auto& u : updates)
        {
            if (u.state.imm_state && u.isDebug() && u.source_id.has_value())
            {
                imm_annotations[ u.uniqueID() ] = createIMMAnnotation(u, estimator, phandler);
            }
        }
    }
    
    std::map<Key, RTSAnnotation> rts_annotations;
    if (rts_debug_infos)
    {
        for (const auto& rts_info : *rts_debug_infos)
            rts_annotations[ rts_info.first ] = createRTSAnnotation(rts_info.second, estimator, phandler);
    }

    addAnnotationData(name, 
                      style, 
                      style_osg, 
                      annos, 
                      fail_pos,
                      skip_pos,
                      &imm_annotations,
                      &rts_annotations);
}

/**
 */
void ReferenceCalculatorAnnotations::addAnnotationData(const std::string& name,
                                                       const AnnotationStyle& style,
                                                       const boost::optional<AnnotationStyle>& style_osg,
                                                       const std::vector<reconstruction::Measurement>& measurements)
{
    std::vector<boost::optional<Eigen::Vector2d>> speed_positions, accel_positions;
    reconstruction::KalmanEstimator::extractVelAccPositionsWGS84(speed_positions, accel_positions, measurements);

    std::vector<TRAnnotation> annos;
    annos.reserve(measurements.size());

    const auto* slice = reconstructor_ ? &reconstructor_->currentSlice() : nullptr;

    for (size_t i = 0; i < measurements.size(); ++i)
    {
        const auto& mm = measurements[ i ];

        if (slice && !slice->inSlice(mm.t))
            continue;

        std::string info;
        std::string tr_info;
        std::string dbc_info;
        std::string sensor_info;
        std::string additional_info;

        if (mm.source_id.has_value())
        {
            //target report info
            tr_info = std::to_string(mm.source_id.value());

            //dbcontent info
            auto dbc_id = Utils::Number::recNumGetDBContId(mm.source_id.value());
            if (COMPASS::instance().dbContentManager().existsDBContentWithId(dbc_id))
                dbc_info = COMPASS::instance().dbContentManager().dbContentWithId(dbc_id);
        
            //sensor info
            auto rec_info = reconstructor_->getInfo(mm.source_id.value());
            if (rec_info)
            {
                if (COMPASS::instance().dataSourceManager().hasDBDataSource(rec_info->ds_id_))
                {
                    const auto& ds = COMPASS::instance().dataSourceManager().dbDataSource(rec_info->ds_id_);
                    sensor_info = ds.hasShortName() ? ds.shortName() : ds.name();
                }

                //get dbcontent string if not yet recovered from record number
                if (dbc_info.empty())
                    dbc_info = COMPASS::instance().dbContentManager().dbContentWithId(rec_info->dbcont_id_);
            }
        }

        //additional info
        additional_info = "ts " + Utils::Time::toString(mm.t) + " interp " + std::to_string(mm.mm_interp);

        if (tr_info.empty()    ) tr_info     = "<unknown tr>";
        if (dbc_info.empty()   ) dbc_info    = "<unknown dbcontent>";
        if (sensor_info.empty()) sensor_info = "<unknown sensor>";

        info = tr_info + "\n" + dbc_info + "\n" + sensor_info + "\n" + additional_info;

        auto a = createTRAnnotation(mm, 
                                    speed_positions[ i ], 
                                    accel_positions[ i ],
                                    {},
                                    info);

        annos.push_back(a);
    }

    addAnnotationData(name, 
                      style, 
                      style_osg, 
                      annos,
                      nullptr,
                      nullptr,
                      nullptr,
                      nullptr);
}

/**
 */
void ReferenceCalculatorAnnotations::addAnnotationData(const std::string& name,
                                                       const AnnotationStyle& style,
                                                       const boost::optional<AnnotationStyle>& style_osg,
                                                       const std::vector<TRAnnotation>& annotations,
                                                       const std::vector<QPointF>* fail_pos,
                                                       const std::vector<QPointF>* skip_pos,
                                                       const std::map<Key, IMMAnnotation>* imm_annotations,
                                                       const std::map<Key, RTSAnnotation>* rts_annotations)
{
    AnnotationData& data = annotation_data_[ name ];

    data.name      = name;
    data.style     = style;
    data.style_osg = style_osg.has_value() ? style_osg.value() : style;

    size_t n     = annotations.size();
    size_t n_cur = data.annotations.size();

    bool added = false;

    for (size_t i = 0; i < n; ++i)
    {
        const auto& a = annotations.at(i);
        
        added = true;

        data.annotations.push_back(a);
    }

    if (fail_pos)
    {
        for (const auto& fp : *fail_pos)
            data.fail_positions.emplace_back(fp.x(), fp.y());
    }
    if (skip_pos)
    {
        for (const auto& sp : *skip_pos)
            data.skip_positions.emplace_back(sp.x(), sp.y());
    }
    // if (interp_positions)
    // {
    //     data.interp_input_positions.insert(data.interp_input_positions.end(), interp_positions->begin(), interp_positions->end());
    // }
    if (imm_annotations)
    {
        //overwrite any updated keys
        for (const auto& a : *imm_annotations)
            data.imm_annotations[ a.first ] = a.second;
    }
    if (rts_annotations)
    {
        //overwrite any updated keys
        for (const auto& a : *rts_annotations)
            data.rts_annotations[ a.first ] = a.second;
    }

    if (added)
        data.slice_begins.push_back(n_cur);
}

/**
 */
void ReferenceCalculatorAnnotations::createAnnotations(ViewPointGenAnnotation* annotation) const
{
    traced_assert(annotation);

    auto parent_anno = annotation;
    auto common_anno = parent_anno->getOrCreateAnnotation("Common");

    const std::string PlotGroup = "Final Reconstruction";

    auto feat_speed_scatter = new ViewPointGenFeatureScatterSeries(ScatterSeriesCollection(), 
        PlotMetadata("Reconstruction", "Speed Common", "Timestamp", "Speed", PlotGroup));
    feat_speed_scatter->scatterSeries().setUseConnectionLines(true);
    common_anno->addFeature(feat_speed_scatter);

    auto scaleColor = [ & ] (const QColor& color, double factor)
    {
        double f = std::max(0.0, factor);
        return QColor(std::min(255, (int)(color.red()   * f)),
                      std::min(255, (int)(color.green() * f)),
                      std::min(255, (int)(color.blue()  * f)));
    };

    for (auto it = annotation_data_.begin(); it != annotation_data_.end(); ++it)
    {
        const auto& data = it->second;

        auto data_anno = parent_anno->getOrCreateAnnotation(data.name);

        const auto& style     = data.style;
        const auto& style_osg = data.style_osg;

        //const QColor ColorBrightest = scaleColor(style.base_color_, 1.5);
        const QColor ColorBright    = scaleColor(style.base_color_, 1.3);
        //const QColor ColorDark      = scaleColor(style.base_color_, 0.7);
        //const QColor ColorDarkest   = scaleColor(style.base_color_, 0.5);

        size_t na = data.annotations.size();

        //add positions
        {
            auto anno = data_anno->getOrCreateAnnotation("Positions");

            std::vector<Eigen::Vector2d> positions(na);
            for (size_t i = 0; i < na; ++i)
                positions[ i ] = data.annotations[ i ].pos_wgs84;

            //point feature
            auto fp = new ViewPointGenFeaturePoints(ViewPointGenFeaturePoints::Symbol::Cross, style_osg.point_size_, positions, {}, false);
            fp->setColor(style_osg.base_color_);
            anno->addFeature(fp);

            //line feature
            auto fl = new ViewPointGenFeatureLineString(false, style_osg.line_width_, ViewPointGenFeatureLineString::LineStyle::Solid, positions, {}, false);
            fl->setColor(style_osg.base_color_);
            anno->addFeature(fl);
        }

        //add connections to input target reports
        {
            auto anno = data_anno->getOrCreateAnnotation("Input Data Connections", true);

            auto anno_corr   = anno->getOrCreateAnnotation("Corrected", true);
            auto anno_uncorr = anno->getOrCreateAnnotation("Uncorrected", false);

            std::vector<Eigen::Vector2d> conn_lines_corr;
            std::vector<Eigen::Vector2d> conn_lines_uncorr;

            for (size_t i = 0; i < na; ++i)
            {
                if (data.annotations[ i ].pos_mm.has_value())
                {
                    conn_lines_corr.push_back(data.annotations[ i ].pos_wgs84);
                    conn_lines_corr.push_back(data.annotations[ i ].pos_mm.value());
                }
                if (data.annotations[ i ].pos_mm_uncorr.has_value())
                {
                    conn_lines_uncorr.push_back(data.annotations[ i ].pos_wgs84);
                    conn_lines_uncorr.push_back(data.annotations[ i ].pos_mm_uncorr.value());
                }
            }

            auto f_corr = new ViewPointGenFeatureLines(style_osg.line_width_, ViewPointGenFeatureLineString::LineStyle::Dotted, conn_lines_corr, {}, false);
            f_corr->setColor(style_osg.base_color_);
            anno_corr->addFeature(f_corr);

            auto f_uncorr = new ViewPointGenFeatureLines(style_osg.line_width_, ViewPointGenFeatureLineString::LineStyle::Dotted, conn_lines_uncorr, {}, false);
            f_uncorr->setColor(style_osg.base_color_);
            anno_uncorr->addFeature(f_uncorr);
        }

        //add velocities
        {
            auto anno = data_anno->getOrCreateAnnotation("Velocities", true);

            std::vector<Eigen::Vector2d> speed_lines;
            std::vector<double>          timestamps;
            std::vector<double>          values;

            for (size_t i = 0; i < na; ++i)
            {
                const auto& a = data.annotations[ i ];

                if (a.speed_pos.has_value())
                {
                    speed_lines.push_back(a.pos_wgs84);
                    speed_lines.push_back(a.speed_pos.value());

                    timestamps.push_back(Utils::Time::toLong(a.ts));
                    values.push_back(a.speed.value());
                }
            }

            auto f = new ViewPointGenFeatureLines(style_osg.line_width_, ViewPointGenFeatureLineString::LineStyle::Solid, speed_lines, {}, false);
            f->setColor(style_osg.base_color_);
            anno->addFeature(f);

            //add histogram data series
            {
                //add to common feature
                RawHistogram h;
                HistogramInitializerT<double> init;
                if (init.createRAW(h, values, true, 20))
                {
                    auto f = new ViewPointGenFeatureHistogram(h, data.name, style.base_color_, {},
                        PlotMetadata("Reconstruction", "Speed " + data.name, "Speed", "", PlotGroup));
                    anno->addFeature(f);
                }
            }

            //add scatter data series
            {
                //add to common feature
                ScatterSeries series;
                for (size_t i = 0; i < timestamps.size(); ++i)
                    series.points.emplace_back(timestamps[ i ], values[ i ]);

                series.data_type_x = ScatterSeries::DataType::DataTypeTimestamp;

                feat_speed_scatter->scatterSeries().addDataSeries(series, data.name, style.base_color_, style.point_size_);

                //add own feature
                auto f = new ViewPointGenFeatureScatterSeries(series, data.name, style.base_color_, style.point_size_, {},
                    PlotMetadata("Reconstruction", "Speed " + data.name, "Timestamp", "Speed", PlotGroup));
                anno->addFeature(f);
            }
        }

        //add accelerations
        {
            auto anno = data_anno->getOrCreateAnnotation("Accelerations", true);

            std::vector<Eigen::Vector2d> accel_lines;

            for (size_t i = 0; i < na; ++i)
            {
                const auto& a = data.annotations[ i ];

                if (a.acc_pos.has_value())
                {
                    accel_lines.push_back(a.pos_wgs84);
                    accel_lines.push_back(a.acc_pos.value());
                }
            }

            auto f = new ViewPointGenFeatureLines(style_osg.line_width_, ViewPointGenFeatureLineString::LineStyle::Solid, accel_lines, {}, false);
            f->setColor(style_osg.base_color_);
            anno->addFeature(f);
        }

        //add accuracies
        {
            auto anno = data_anno->getOrCreateAnnotation("Accuracies", true);

            std::vector<Eigen::Vector2d> positions;
            std::vector<Eigen::Vector3d> accuracies;

            double lat_min = std::numeric_limits<double>::max();
            double lat_max = std::numeric_limits<double>::lowest();
            double lon_min = std::numeric_limits<double>::max();
            double lon_max = std::numeric_limits<double>::lowest();

            for (size_t i = 0; i < na; ++i)
            {
                const auto& a = data.annotations[ i ];

                if (a.accuracy.has_value())
                {
                    const auto& pos = a.pos_wgs84;

                    positions.push_back(pos);
                    accuracies.push_back(a.accuracy.value());

                    if (pos.x() < lat_min) lat_min = pos.x();
                    if (pos.x() > lat_max) lat_max = pos.x();
                    if (pos.y() < lon_min) lon_min = pos.y();
                    if (pos.y() > lon_max) lon_max = pos.y();
                }
            }

            auto f = new ViewPointGenFeatureErrEllipses(style_osg.line_width_, 32u, positions, {}, accuracies, false);
            f->setColor(style_osg.base_color_);
            anno->addFeature(f);

            //just for fun: add a grid
            QRectF roi(lon_min, lat_min, lon_max - lon_min, lat_max - lat_min);

            if (!roi.isEmpty())
            {
#if 0
                auto anno = data_anno->getOrCreateAnnotation("Accuracy Grid");

                Grid2D grid;
                grid.create(roi, grid2d::GridResolution().setCellCount(100, 100));
                
                for (size_t i = 0; i < positions.size(); ++i)
                    grid.addValue(positions[ i ].y(), positions[ i ].x(), std::max(accuracies[ i ].x(), accuracies[ i ].y()));

                Grid2DLayers layers;
                grid.addToLayers(layers, "min accuracy", grid2d::ValueType::ValueTypeMax);

                Grid2DRenderSettings rsettings;
                rsettings.pixels_per_cell = 10;
                rsettings.min_value       = 0.0;
                rsettings.max_value       = 10.0;

                rsettings.color_map.create(ColorMap::ColorScale::Green2Red, 10);
                
                auto result = Grid2DLayerRenderer::render(layers.layer(0), rsettings);

                auto f = new ViewPointGenFeatureGeoImage(result.first, result.second);
                anno->addFeature(f);
#endif
            }
        }

        //add process noise graph
        {
            auto anno = data_anno->getOrCreateAnnotation("Additional Infos", true);

            ScatterSeries series;
            for (size_t i = 0; i < na; ++i)
            {
                const auto& a = data.annotations[ i ];

                if (a.Q_var.has_value())
                    series.points.emplace_back(Utils::Time::toLong(a.ts), a.Q_var.value());
            }
            series.points.emplace_back(series.points.back().x(), 0.0);
            series.data_type_x = ScatterSeries::DataType::DataTypeTimestamp;

            //add own feature
            auto f = new ViewPointGenFeatureScatterSeries(series, data.name, style.base_color_, style.point_size_, {},
                PlotMetadata("Reconstruction", "Used Q_vars " + data.name, "Timestamp", "Q_var", PlotGroup));
            anno->addFeature(f);
        }

        //add special positions
        {
            std::vector<Eigen::Vector2d> positions;
            for (size_t i = 0; i < na; ++i)
                if (data.annotations[ i ].reset)
                    positions.push_back(data.annotations[ i ].pos_wgs84);

            if (!positions.empty())
            {
                auto anno = data_anno->getOrCreateAnnotation("Reset Positions", true);

                auto fp = new ViewPointGenFeaturePoints(ViewPointGenFeaturePoints::Symbol::BorderThick, style_osg.point_size_ * 1.5f, positions, {}, false);
                fp->setColor(ColorBright);
                anno->addFeature(fp);
            }
        }
        {
            std::vector<Eigen::Vector2d> positions;
            for (size_t i = 0; i < na; ++i)
                if (data.annotations[ i ].proj_change)
                    positions.push_back(data.annotations[ i ].pos_wgs84);

            if (!positions.empty())
            {
                auto anno = data_anno->getOrCreateAnnotation("Projection Change Positions", true);

                auto fp = new ViewPointGenFeaturePoints(ViewPointGenFeaturePoints::Symbol::BorderThick, style_osg.point_size_ * 1.5f, positions, {}, false);
                fp->setColor(ColorBright);
                anno->addFeature(fp);
            }
        }

        //add fail positions
        if (!data.fail_positions.empty())
        {
            auto anno = data_anno->getOrCreateAnnotation("Fail Positions", true);

            auto fp = new ViewPointGenFeaturePoints(ViewPointGenFeaturePoints::Symbol::BorderThick, style_osg.point_size_ * 1.5f, data.fail_positions, {}, false);
            fp->setColor(ColorBright);
            anno->addFeature(fp);
        }

        //add skip positions
        if (!data.skip_positions.empty())
        {
            auto anno = data_anno->getOrCreateAnnotation("Skip Positions", true);

            auto fp = new ViewPointGenFeaturePoints(ViewPointGenFeaturePoints::Symbol::BorderThick, style_osg.point_size_ * 1.5f, data.skip_positions, {}, false);
            fp->setColor(ColorBright);
            anno->addFeature(fp);
        }

        //add interpolated input positions
        if (!data.interp_input_positions.empty())
        {
            auto anno = data_anno->getOrCreateAnnotation("Interpolated Input Positions", true);

            auto fp = new ViewPointGenFeaturePoints(ViewPointGenFeaturePoints::Symbol::Circle, style_osg.point_size_ * 1.5f, data.interp_input_positions, {}, false);
            fp->setColor(ColorBright);
            anno->addFeature(fp);
        }

        //add slice begin positions
        {
            auto anno = data_anno->getOrCreateAnnotation("Slice Begins", true);

            std::vector<Eigen::Vector2d> positions;
            for (auto idx : data.slice_begins)
                positions.push_back(data.annotations[ idx ].pos_wgs84);

            auto fp = new ViewPointGenFeaturePoints(ViewPointGenFeaturePoints::Symbol::BorderThick, style_osg.point_size_ * 1.5f, positions, {}, false);
            fp->setColor(ColorBright);
            anno->addFeature(fp);
        }

        //add textual info
        // {
        //     auto anno = data_anno->getOrCreateAnnotation("Info Labels", true);

        //     for (size_t i = 0; i < na; ++i)
        //     {
        //         const auto& a = data.annotations[ i ];

        //         if (!a.info.has_value())
        //             continue;

        //         auto feat = new ViewPointGenFeatureText(a.info.value(), a.pos_wgs84.x(), a.pos_wgs84.y());
        //         feat->setColor(ColorBright);

        //         anno->addFeature(feat);
        //     }
        // }

        //add rts debug annotations
        {
            auto anno_rts = data_anno->getOrCreateAnnotation("RTS Infos", true);

            loginf << "adding " << data.rts_annotations.size() << " RTS info(s)";

            size_t cnt = 0;
            for (const auto& rts_anno : data.rts_annotations)
            {
                std::string name = "RTSInfo" + std::to_string(cnt) + (rts_anno.second.extra_info.empty() ? "" : ": " + rts_anno.second.extra_info);

                auto anno_info = anno_rts->getOrCreateAnnotation(name);

                std::vector<Eigen::Vector2d> positions_smooth;
                std::vector<Eigen::Vector2d> positions_nonsmooth;
                std::vector<Eigen::Vector2d> connections;
                std::vector<QColor> colors;

                //smoothed position
                positions_smooth.push_back(rts_anno.second.rts_step.state0_smooth.pos_wgs84);
                positions_nonsmooth.push_back(rts_anno.second.rts_step.state0.pos_wgs84);
                connections.push_back(positions_smooth.back());
                connections.push_back(positions_nonsmooth.back());
                colors.push_back(rts_anno.second.rts_step.color);

                //smoothed submodel positions
                for (const auto& rts_anno_model : rts_anno.second.rts_step_models)
                {
                    positions_smooth.push_back(rts_anno_model.state0_smooth.pos_wgs84);
                    positions_nonsmooth.push_back(rts_anno_model.state0.pos_wgs84);
                    connections.push_back(positions_smooth.back());
                    connections.push_back(positions_nonsmooth.back());
                    colors.push_back(rts_anno_model.color);
                }

                auto anno_smooth = anno_info->getOrCreateAnnotation("Smooth Positions");

                auto fp_smooth = new ViewPointGenFeaturePoints(ViewPointGenFeaturePoints::Symbol::Cross, style_osg.point_size_, positions_smooth, colors, true);
                anno_smooth->addFeature(fp_smooth);

                auto anno_input = anno_info->getOrCreateAnnotation("Input Positions");

                auto fp_nonsmooth = new ViewPointGenFeaturePoints(ViewPointGenFeaturePoints::Symbol::Circle, style_osg.point_size_ * 0.7, positions_nonsmooth, colors, true);
                anno_input->addFeature(fp_nonsmooth);

                auto fl = new ViewPointGenFeatureLines(style_osg.line_width_, ViewPointGenFeatureLineString::LineStyle::Dotted, connections, {}, true);
                fl->setColor(QColor(200, 200, 200));
                anno_input->addFeature(fl);

                ++cnt;
            }
        }

        //add imm debug annotations
        {
            auto anno_rts = data_anno->getOrCreateAnnotation("IMM Infos", true);

            loginf << "adding " << data.imm_annotations.size() << " IMM info(s)";

            size_t cnt = 0;
            for (const auto& imm_anno : data.imm_annotations)
            {
                std::string name = "IMMInfo" + std::to_string(cnt) + (imm_anno.second.extra_info.empty() ? "" : ": " + imm_anno.second.extra_info);

                auto anno_info = anno_rts->getOrCreateAnnotation(name);

                std::vector<Eigen::Vector2d> positions;
                std::vector<QColor> colors;

                //position
                positions.push_back(imm_anno.second.imm_step.state.pos_wgs84);
                colors.push_back(imm_anno.second.imm_step.color);

                //submodel positions
                for (const auto& imm_anno_model : imm_anno.second.imm_step_models)
                {
                    positions.push_back(imm_anno_model.state.pos_wgs84);
                    colors.push_back(imm_anno_model.color);
                }

                auto fp = new ViewPointGenFeaturePoints(ViewPointGenFeaturePoints::Symbol::Cross, style_osg.point_size_, positions, colors, true);
                anno_info->addFeature(fp);

                ++cnt;
            }
        }
    }
}

}
