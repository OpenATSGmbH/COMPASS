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

#include "catch.hpp"
#include "taskresult.h"
#include "itaskprovider.h"
#include "report/report.h"
#include "report/section.h"
#include "report/sectioncontentfigure.h"
#include "reportfigure.h"
#include "viewpointgenerator.h"

#include "json.hpp"

#include <QImage>

#include <stdexcept>

namespace
{
    /**
     * Task provider of a result that is not stored and not shown. Each use of it is an error:
     * a figure that is drawn by render widgets needs no manager and no view.
     */
    class NoTaskProvider : public ITaskProvider
    {
    public:
        AppState& appState() override { unexpected(); }
        DBInterface& dbInterface() override { unexpected(); }
        DBContentManager& dbContentManager() override { unexpected(); }
        LicenseManager& licenseManager() override { unexpected(); }
        ViewManager& viewManager() override { unexpected(); }

        void beginTaskResultWriting(const std::string&, task::TaskResultType, bool) override { unexpected(); }
        bool hasCurrentResult() const override { unexpected(); }
        std::shared_ptr<TaskResult>& currentResult() override { unexpected(); }
        std::shared_ptr<ResultReport::Report>& currentReport() override { unexpected(); }
        void endTaskResultWriting(bool, bool) override { unexpected(); }

        std::shared_ptr<TaskResult> result(const std::string&) const override { unexpected(); }
        bool hasResult(const std::string&) const override { unexpected(); }
        std::shared_ptr<TaskResult> createResult(unsigned int, task::TaskResultType) override { unexpected(); }

        void resultHeaderChanged(const TaskResult&) override { unexpected(); }
        void resultContentChanged(const TaskResult&) override { unexpected(); }

        void setViewableDataConfig(const nlohmann::json::object_t&, bool, bool) override { unexpected(); }
        void unsetViewableDataConfig() override { unexpected(); }

        std::shared_ptr<ResultReport::SectionContent> loadContent(ResultReport::Section*, unsigned int, bool) const override { unexpected(); }
        ResultT<nlohmann::json> loadResultContent(unsigned int) const override { unexpected(); }

        void storeBackupSection() override { unexpected(); }
        void restoreBackupSection() override { unexpected(); }

        void selectResultID(const std::string&, bool) override { unexpected(); }

    private:
        [[noreturn]] static void unexpected() { throw std::logic_error("unexpected use of the task provider"); }
    };

    /// view point with one histogram annotation
    nlohmann::json histogramViewPoint(bool loads_data)
    {
        RawHistogram h;
        h.addBin(RawHistogramBin(3, "0-10"));
        h.addBin(RawHistogramBin(12, "10-20"));

        ViewPointGenVP vp("Histogram", 0, "Histogram");
        if (loads_data)
            vp.enableDataLoading();
        vp.annotations().getOrCreateAnnotation("Errors")->addFeature(
            new ViewPointGenFeatureHistogram(h, "Errors", Qt::blue, boost::optional<bool>(),
                                             PlotMetadata("Errors", "", "Error [m]", "Count")));

        nlohmann::json j;
        vp.toJSON(j);

        return j;
    }

    /// view point without load_data with a points annotation, which no render widget draws
    nlohmann::json pointsViewPoint()
    {
        ViewPointGenVP vp("Points", 0, "Points");
        vp.annotations().getOrCreateAnnotation("Positions")->addFeature(
            new ViewPointGenFeaturePoints(ViewPointGenFeaturePoints::Symbol::Circle, 6.0f,
                                          { Eigen::Vector2d(47.0, 16.0) }));

        nlohmann::json j;
        vp.toJSON(j);

        return j;
    }

    /// task result that gives each on-demand figure the view point on_demand_view_point when it is loaded
    class OnDemandTaskResult : public TaskResult
    {
    public:
        explicit OnDemandTaskResult(ITaskProvider& provider) : TaskResult(1, provider) {}

        nlohmann::json        on_demand_view_point;
        mutable unsigned int  num_loads = 0;

    protected:
        bool loadOnDemandFigure_impl(ResultReport::SectionContentFigure* figure) const override
        {
            ++num_loads;

            auto content = std::make_shared<nlohmann::json::object_t>(
                on_demand_view_point.get<nlohmann::json::object_t>());
            figure->setViewableFunc([ content ] () { return content; });

            return true;
        }
    };

    struct ReportWithSection
    {
        ReportWithSection() : result(provider), section(result.report()->getSection("Figures")) {}

        NoTaskProvider         provider;
        OnDemandTaskResult     result;
        ResultReport::Section& section;
    };

    using FigureKind = ResultReport::SectionContentFigure::FigureKind;
}

TEST_CASE("report figure with a histogram view point is a simple figure", "[result][reportfigure]")
{
    ReportWithSection report;

    auto& figure = report.section.addFigure("Errors Histogram",
                                            ResultReport::SectionContentViewable(histogramViewPoint(false)));

    const auto classification = figure.classify();

    CHECK(classification.kind == FigureKind::Simple);
    REQUIRE(classification.render_entries.size() == 1);
    CHECK(classification.render_entries[ 0 ].image_name == "histogram_0");

    SECTION("the export gives one image per annotation and uses no view")
    {
        //the task provider throws on a view manager request
        const auto images = report.result.renderFigure(figure);

        REQUIRE(images.size() == 1);

        CHECK(images[ 0 ].second == "histogram_0");
        CHECK(images[ 0 ].first.size() == QSize(2400, 1600));
    }
}

TEST_CASE("report figure that loads data is a view figure", "[result][reportfigure]")
{
    ReportWithSection report;

    auto& figure = report.section.addFigure("Errors Histogram",
                                            ResultReport::SectionContentViewable(histogramViewPoint(true)));

    const auto classification = figure.classify();

    CHECK(classification.kind == FigureKind::View);
    CHECK(classification.render_entries.empty());

    //the task provider throws on a view manager request
    CHECK(report.result.renderFigure(figure).empty());
}

TEST_CASE("report figure on demand is classified after its content is loaded", "[result][reportfigure]")
{
    ReportWithSection report;

    auto& figure = report.section.addFigure("Errors Histogram", ResultReport::SectionContentViewable());
    figure.setOnDemand();

    SECTION("a simple figure gives images")
    {
        report.result.on_demand_view_point = histogramViewPoint(false);

        const auto classification = figure.classify();

        CHECK(report.result.num_loads == 1);
        CHECK(classification.kind == FigureKind::Simple);
        CHECK(classification.render_entries.size() == 1);

        const auto images = report.result.renderFigure(figure);

        REQUIRE(images.size() == 1);
        CHECK(images[ 0 ].second == "histogram_0");
        CHECK(report.result.num_loads == 1);
    }

    SECTION("a view figure gives no images")
    {
        report.result.on_demand_view_point = histogramViewPoint(true);

        CHECK(figure.classify().kind == FigureKind::View);
        CHECK(report.result.renderFigure(figure).empty());
    }
}

TEST_CASE("report figure without load_data that no render widget draws cannot be drawn", "[result][reportfigure]")
{
    ReportWithSection report;

    auto& figure = report.section.addFigure("Positions",
                                            ResultReport::SectionContentViewable(pointsViewPoint()));

    const auto classification = figure.classify();

    CHECK(classification.kind == FigureKind::CannotBeDrawn);
    CHECK(classification.render_entries.empty());

    //the task provider throws on a view manager request
    CHECK(report.result.renderFigure(figure).empty());
}
