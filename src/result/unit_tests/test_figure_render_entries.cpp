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

    struct ReportWithSection
    {
        ReportWithSection() : result(1, provider), section(result.report()->getSection("Figures")) {}

        NoTaskProvider         provider;
        TaskResult             result;
        ResultReport::Section& section;
    };
}

TEST_CASE("report figure with a histogram view point is drawn by a render widget", "[result][reportfigure]")
{
    ReportWithSection report;

    auto& figure = report.section.addFigure("Errors Histogram",
                                            ResultReport::SectionContentViewable(histogramViewPoint(false)));

    const auto entries = figure.renderEntries();

    REQUIRE(entries.size() == 1);
    CHECK(entries[ 0 ].image_name == "histogram_0");

    SECTION("the export gives one image per annotation and uses no view")
    {
        //the task provider throws on a view manager request
        const auto images = report.result.renderFigure(figure);

        REQUIRE(images.size() == 1);

        CHECK(images[ 0 ].second == "histogram_0");
        CHECK(images[ 0 ].first.size() == QSize(2400, 1600));
    }
}

TEST_CASE("report figure that loads data is shown by the views", "[result][reportfigure]")
{
    ReportWithSection report;

    auto& figure = report.section.addFigure("Errors Histogram",
                                            ResultReport::SectionContentViewable(histogramViewPoint(true)));

    CHECK(figure.renderEntries().empty());
}

TEST_CASE("report figure on demand is shown by the views", "[result][reportfigure]")
{
    ReportWithSection report;

    bool content_built = false;

    ResultReport::SectionContentViewable viewable([ & ] ()
    {
        content_built = true;

        auto content = std::make_shared<nlohmann::json::object_t>();
        *content = histogramViewPoint(false);

        return content;
    });

    auto& figure = report.section.addFigure("Errors Histogram", viewable);
    figure.setOnDemand();

    CHECK(figure.renderEntries().empty());

    //the content is not built only to decide how the figure is drawn
    CHECK(!content_built);
}
