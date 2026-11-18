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

#include "report/reportdocumentview.h"
#include "report/report.h"
#include "report/section.h"

#include "logger.h"
#include "traced_assert.h"

#include <QApplication>
#include <QEvent>
#include <QLabel>
#include <QResizeEvent>
#include <QScrollBar>
#include <QStyle>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <chrono>
#include <cstdlib>

namespace ResultReport
{

namespace
{
    const int    HeadingTopMargin   = 16;
    const int    ScrollSingleStep   = 20;
    const int    PreloadPollMSec    = 20;
    const size_t MaxPreloadSections = 5;

    class WaitCursor
    {
    public:
        WaitCursor() { QApplication::setOverrideCursor(Qt::WaitCursor); }
        ~WaitCursor() { QApplication::restoreOverrideCursor(); }

        WaitCursor(const WaitCursor&) = delete;
        WaitCursor& operator=(const WaitCursor&) = delete;
    };
}

/**
 */
ReportDocumentView::ReportDocumentView(QWidget* parent)
:   QAbstractScrollArea(parent)
{
    setObjectName("report_document_view");

    // a vertical scroll bar which comes and goes would change the width and clear all measured heights
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    viewport()->setBackgroundRole(QPalette::NoRole);

    verticalScrollBar()->setSingleStep(ScrollSingleStep);
    horizontalScrollBar()->setSingleStep(ScrollSingleStep);

    build_timer_.setSingleShot(true);
    build_timer_.setInterval(0);
    connect(&build_timer_, &QTimer::timeout, this, &ReportDocumentView::buildStep);

    relayout_timer_.setSingleShot(true);
    relayout_timer_.setInterval(0);
    connect(&relayout_timer_, &QTimer::timeout, this, [ this ] () { relayout(false); });

    preload_timer_.setInterval(PreloadPollMSec);
    connect(&preload_timer_, &QTimer::timeout, this, &ReportDocumentView::pollPreload);
}

/**
 * The owner releases the entries while the sections still exist (see clear()). Entries left here
 * only touch their section if the content widget still exists, which the section owns.
 */
ReportDocumentView::~ReportDocumentView()
{
    discardPreload();
    releaseAll();
}

/**
 */
void ReportDocumentView::setRootSection(Section* root, const ReportDocumentLayout::SectionSet& collapsed)
{
    clear();

    if (!root)
        return;

    {
        ReportDocumentLayout::Entry probe;
        probe.number = "1";

        std::unique_ptr<QWidget> heading(createEntryWidget(probe, nullptr));
        layout_.setHeadingHeight(heading->sizeHint().height());
    }

    layout_.setRootSection(root, collapsed);

    updateScrollRange();
    setScrollValue(0);
    updateCurrentSection();

    build_timer_.start();
}

/**
 * Builds the document again with other collapsed sections. The built entries which stay keep their
 * widgets, and the point at the top of the viewport stays in place. If its section left the document,
 * the nearest parent in the document moves to the top.
 */
void ReportDocumentView::setCollapsedSections(const ReportDocumentLayout::SectionSet& collapsed)
{
    if (!layout_.rootSection() || layout_.empty())
        return;

    const int    scroll       = verticalScrollBar()->value();
    const size_t anchor_index = layout_.indexAt(scroll);
    Section*     anchor       = layout_.entry(anchor_index).section;
    int          anchor_delta = scroll - layout_.offsetOf(anchor_index);

    std::vector<std::pair<Section*, BuiltEntry>> built;
    for (const auto& b : built_)
        built.emplace_back(layout_.entry(b.first).section, b.second);

    built_.clear();

    layout_.setCollapsedSections(collapsed);

    //the entry indexes changed, a step in a nested event loop must not use its old ones
    ++generation_;

    for (const auto& b : built)
    {
        auto index = layout_.indexOf(b.first);

        if (index.has_value())
            built_[ index.value() ] = b.second;
        else
            releaseBuiltEntry(b.first, b.second);
    }

    std::optional<size_t> new_anchor_index;
    for (Section* s = anchor; s && !new_anchor_index.has_value(); s = s->parentSection())
    {
        new_anchor_index = layout_.indexOf(s);
        if (!new_anchor_index.has_value())
            anchor_delta = 0;
    }

    updateScrollRange();

    if (new_anchor_index.has_value())
        setScrollValue(layout_.offsetOf(new_anchor_index.value()) + anchor_delta);

    positionEntries();

    if (!layout_.indexOf(current_section_).has_value())
        updateCurrentSection();

    build_timer_.start();
}

/**
 */
void ReportDocumentView::clear()
{
    //the worker uses the sections, which the caller may delete after the clear
    discardPreload();

    releaseAll();

    layout_.clear();
    ++generation_;

    pending_target_.reset();
    scroll_up_ = false;

    updateScrollRange();
    setCurrentSection(nullptr);
}

/**
 * Shows the target at the top of the viewport and builds it at once, the sections around it follow
 * in later build steps.
 */
void ReportDocumentView::scrollToSection(Section* section,
                                         int offset_in_section,
                                         bool preload_ondemand_contents)
{
    auto index = layout_.indexOf(section);
    if (!index.has_value())
    {
        logwrn << "section '" << (section ? section->id() : std::string()) << "' not in document";
        return;
    }

    Target target;
    target.section                   = section;
    target.offset                    = std::max(0, offset_in_section);
    target.preload_ondemand_contents = preload_ondemand_contents;

    pending_target_ = target;

    setScrollValue(layout_.offsetOf(index.value()) + target.offset);
    positionEntries();
    setCurrentSection(section);

    //a step in progress (nested event loop of a content widget) schedules the next one
    buildStep();
}

/**
 */
Section* ReportDocumentView::currentSection() const
{
    return current_section_;
}

/**
 * Offset of the viewport top inside the current section.
 */
int ReportDocumentView::offsetInCurrentSection() const
{
    if (!current_section_)
        return 0;

    auto index = layout_.indexOf(current_section_);
    if (!index.has_value())
        return 0;

    return std::max(0, verticalScrollBar()->value() - layout_.offsetOf(index.value()));
}

/**
 * Builds the next entry near the visible area, then returns to the event loop, so scrolling goes on
 * while the document is built. Each step picks its entry from the current scroll position. Once all
 * entries near the visible area are built, frees the far entries and preloads the next contents in
 * the background. Runs deferred or from scrollToSection(), never inside another step.
 */
void ReportDocumentView::buildStep()
{
    //called from a nested event loop of a content widget: the running step schedules the next one
    if (building_ || layout_.empty())
        return;

    auto index = nextEntryToBuild();

    if (!index.has_value())
    {
        pending_target_.reset();

        releaseFarEntries();
        startPreload();
        return;
    }

    building_ = true;

    const unsigned int generation = generation_;

    Section* section = layout_.entry(index.value()).section;

    if (!section->allContentLoaded())
    {
        WaitCursor wait_cursor;

        //the preload may be loading it, take over its results instead of loading twice
        if (preload_.has_value())
        {
            const auto& in_flight = preload_->sections;

            if (std::find(in_flight.begin(), in_flight.end(), section) != in_flight.end())
                finishPreload();
        }

        section->loadAllContent();
    }

    const bool preload_ondemand_contents = pending_target_.has_value() &&
                                           pending_target_->section == section &&
                                           pending_target_->preload_ondemand_contents;

    buildEntry(index.value(), preload_ondemand_contents);

    //else the document was reset during a nested event loop
    if (generation == generation_)
    {
        relayout(false);
        applyPendingTarget();

        //during a long scroll the build window is never complete, free behind the user as well
        releaseFarEntries();
    }

    building_ = false;

    build_timer_.start();
}

/**
 * The entry which the next build step builds: the first unbuilt entry in the viewport, else the
 * nearest unbuilt entry within one viewport height above and below.
 */
std::optional<size_t> ReportDocumentView::nextEntryToBuild() const
{
    const int top    = verticalScrollBar()->value();
    const int height = viewport()->height();

    for (size_t i : layout_.buildOrder(top, top + height, height))
        if (!built_.count(i))
            return i;

    return {};
}

/**
 * The entries above the target get measured while the document is built, the target stays at the
 * top of the viewport until the build is done or the user scrolls.
 */
void ReportDocumentView::applyPendingTarget()
{
    if (!pending_target_.has_value())
        return;

    auto index = layout_.indexOf(pending_target_->section);
    if (!index.has_value())
    {
        pending_target_.reset();
        return;
    }

    setScrollValue(layout_.offsetOf(index.value()) + pending_target_->offset);
    positionEntries();
}

/**
 * First and last entry which overlap the viewport extended by the margin above and below.
 */
std::pair<size_t, size_t> ReportDocumentView::entryRange(int margin) const
{
    const int top    = verticalScrollBar()->value() - margin;
    const int bottom = verticalScrollBar()->value() + viewport()->height() + margin;

    return { layout_.indexAt(std::max(0, top)), layout_.indexAt(std::max(0, bottom - 1)) };
}

/**
 */
void ReportDocumentView::buildEntry(size_t index, bool preload_ondemand_contents)
{
    const unsigned int generation = generation_;

    QWidget* content = layout_.entry(index).section->getContentWidget(preload_ondemand_contents);
    traced_assert(content);

    //the content widget may have run a nested event loop which reset the document
    if (generation != generation_ || built_.count(index))
        return;

    BuiltEntry built;
    built.widget  = createEntryWidget(layout_.entry(index), content);
    built.content = content;

    built.widget->installEventFilter(this);
    built.widget->show();

    built_[ index ] = built;
}

/**
 * Wheel events over the inner scroll areas (tables, text views) go to eventFilter() first.
 * Runs with each measurement, a content widget may add scroll areas later. A filter installed
 * twice is called once.
 */
void ReportDocumentView::watchInnerScrollAreas(QWidget* widget)
{
    for (auto area : widget->findChildren<QAbstractScrollArea*>())
        area->viewport()->installEventFilter(this);
}

/**
 * Entry widget with the numbered heading above the content widget of the section.
 */
QWidget* ReportDocumentView::createEntryWidget(const ReportDocumentLayout::Entry& entry, QWidget* content)
{
    QWidget* widget = new QWidget(viewport());

    QVBoxLayout* layout = new QVBoxLayout(widget);
    layout->setContentsMargins(0, HeadingTopMargin, 0, 0);
    layout->setSpacing(0);

    std::string heading = entry.section ? entry.section->heading() : std::string();
    if (!entry.number.empty())
        heading = entry.number + "  " + heading;

    QLabel* label = new QLabel(QString::fromStdString(heading));

    QFont font = label->font();
    font.setBold(true);
    font.setPointSizeF(font.pointSizeF() * 1.2);
    label->setFont(font);

    const int margin = style()->pixelMetric(QStyle::PM_LayoutLeftMargin);
    label->setContentsMargins(margin, 0, margin, 0);

    layout->addWidget(label);

    if (content)
        layout->addWidget(content);

    return widget;
}

/**
 */
void ReportDocumentView::releaseEntry(size_t index)
{
    auto it = built_.find(index);
    if (it == built_.end())
        return;

    BuiltEntry built = it->second;
    built_.erase(it);

    releaseBuiltEntry(layout_.entry(index).section, built);
}

/**
 */
void ReportDocumentView::releaseBuiltEntry(Section* section, const BuiltEntry& built)
{
    built.widget->removeEventFilter(this);

    //the section owns the content widget: it is alive only as long as the section is
    if (built.content)
        section->releaseContentWidget();

    //deferred, the release may run inside an event handler of a child widget
    built.widget->hide();
    built.widget->deleteLater();
}

/**
 * Frees the entries outside of two viewport heights above and below the visible area.
 */
void ReportDocumentView::releaseFarEntries()
{
    if (layout_.empty())
        return;

    auto range = entryRange(2 * viewport()->height());

    std::vector<size_t> far_entries;
    for (const auto& b : built_)
        if (b.first < range.first || b.first > range.second)
            far_entries.push_back(b.first);

    for (size_t i : far_entries)
        releaseEntry(i);
}

/**
 */
void ReportDocumentView::releaseAll()
{
    while (!built_.empty())
        releaseEntry(built_.begin()->first);
}

/**
 * Loads the contents of the next sections after the build window in scroll direction on a worker
 * thread: the unloaded sections which start within one viewport height, at most MaxPreloadSections.
 */
void ReportDocumentView::startPreload()
{
    if (preload_.has_value() || layout_.empty())
        return;

    const int  view_height = viewport()->height();
    const auto range       = entryRange(view_height);

    std::vector<Section*> sections;

    auto addIfUnloaded = [ & ] (size_t index)
    {
        Section* section = layout_.entry(index).section;
        if (!section->allContentLoaded())
            sections.push_back(section);
    };

    if (scroll_up_)
    {
        const int limit = layout_.offsetOf(range.first) - view_height;

        //offsetOf(i) is the bottom of entry i - 1
        for (size_t i = range.first; i > 0 && layout_.offsetOf(i) > limit && sections.size() < MaxPreloadSections; --i)
            addIfUnloaded(i - 1);
    }
    else
    {
        const int limit = layout_.offsetOf(range.second) + layout_.heightOf(range.second) + view_height;

        for (size_t i = range.second + 1; i < layout_.size() && layout_.offsetOf(i) < limit && sections.size() < MaxPreloadSections; ++i)
            addIfUnloaded(i);
    }

    if (sections.empty())
        return;

    //the worker gets copies of the content IDs, it never reads or writes the content slots of a section
    std::vector<PreloadItem> items;
    for (Section* section : sections)
        for (const auto& c : section->unloadedContentIDs())
            items.push_back({ section, c.first, c.second, nullptr });

    const Report* report = sections.front()->report();
    traced_assert(report);

    logdbg << "preloading " << items.size() << " contents of " << sections.size() << " sections";

    Preload preload;
    preload.sections = sections;
    preload.cancel   = std::make_shared<std::atomic<bool>>(false);
    preload.future   = std::async(std::launch::async, [ report, items, cancel = preload.cancel ] () mutable
    {
        for (auto& item : items)
        {
            if (*cancel)
                break;

            item.content = report->loadContent(item.section, item.id, false);

            //owned by the worker alone until it is installed, it has no widget yet
            if (item.content && !*cancel)
                item.content->loadOnDemandOffMainThread();
        }

        return items;
    });

    preload_ = std::move(preload);
    preload_timer_.start();
}

/**
 */
void ReportDocumentView::pollPreload()
{
    if (!preload_.has_value())
    {
        preload_timer_.stop();
        return;
    }

    if (preload_->future.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        return;

    finishPreload();

    if (nextEntryToBuild().has_value())
        build_timer_.start();
    else
        startPreload();
}

/**
 * Waits for the preload and installs its contents into the sections.
 */
void ReportDocumentView::finishPreload()
{
    if (!preload_.has_value())
        return;

    preload_timer_.stop();

    Preload preload = std::move(preload_.value());
    preload_.reset();

    auto items = preload.future.get();

    //a failed load is not installed, the next build loads it on the main thread
    for (const auto& item : items)
        if (item.content)
            item.section->adoptLoadedContent(item.index, item.content);
}

/**
 * Waits for the preload and drops its contents.
 */
void ReportDocumentView::discardPreload()
{
    if (!preload_.has_value())
        return;

    preload_timer_.stop();

    *preload_->cancel = true;

    preload_->future.wait();
    preload_.reset();
}

/**
 * Measures the built entries and keeps the point at the top of the viewport in place.
 */
void ReportDocumentView::relayout(bool width_changed)
{
    if (layout_.empty())
        return;

    const int    scroll       = verticalScrollBar()->value();
    const size_t anchor       = layout_.indexAt(scroll);
    const int    anchor_delta = scroll - layout_.offsetOf(anchor);

    if (width_changed)
        layout_.clearMeasuredHeights();

    for (const auto& b : built_)
    {
        QWidget* widget = b.second.widget;

        watchInnerScrollAreas(widget);

        const int width  = entryWidth(widget);
        const int height = widget->hasHeightForWidth() ? widget->heightForWidth(width)
                                                       : widget->sizeHint().height();

        const auto& measured = layout_.entry(b.first).measured_height;
        if (!measured.has_value() || measured.value() != height)
            layout_.setMeasuredHeight(b.first, height);
    }

    updateScrollRange();

    const int anchor_height = layout_.heightOf(anchor);
    setScrollValue(layout_.offsetOf(anchor) + std::min(anchor_delta, std::max(0, anchor_height - 1)));

    positionEntries();
}

/**
 */
void ReportDocumentView::positionEntries()
{
    const int x0 = horizontalScrollBar()->value();
    const int y0 = verticalScrollBar()->value();

    for (const auto& b : built_)
    {
        QRect rect(-x0,
                   layout_.offsetOf(b.first) - y0,
                   entryWidth(b.second.widget),
                   layout_.heightOf(b.first));

        if (b.second.widget->geometry() != rect)
            b.second.widget->setGeometry(rect);
    }
}

/**
 */
void ReportDocumentView::updateScrollRange()
{
    const int view_width  = viewport()->width();
    const int view_height = viewport()->height();

    int width = view_width;
    for (const auto& b : built_)
        width = std::max(width, entryWidth(b.second.widget));

    const int height = layout_.empty() ? 0 : layout_.totalHeight();

    bool programmatic = programmatic_scroll_;
    programmatic_scroll_ = true;

    verticalScrollBar()->setPageStep(view_height);
    verticalScrollBar()->setRange(0, std::max(0, height - view_height));

    horizontalScrollBar()->setPageStep(view_width);
    horizontalScrollBar()->setRange(0, std::max(0, width - view_width));

    programmatic_scroll_ = programmatic;
}

/**
 */
void ReportDocumentView::setScrollValue(int value)
{
    bool programmatic = programmatic_scroll_;
    programmatic_scroll_ = true;

    verticalScrollBar()->setValue(value);

    programmatic_scroll_ = programmatic;
}

/**
 */
int ReportDocumentView::entryWidth(QWidget* widget) const
{
    return std::max(viewport()->width(), widget->minimumSizeHint().width());
}

/**
 */
void ReportDocumentView::resizeEvent(QResizeEvent* event)
{
    QAbstractScrollArea::resizeEvent(event);

    const bool width_changed = viewport()->width() != viewport_width_;
    viewport_width_ = viewport()->width();

    relayout(width_changed);
    updateScrollRange();

    build_timer_.start();
}

/**
 */
void ReportDocumentView::scrollContentsBy(int dx, int dy)
{
    positionEntries();

    if (!programmatic_scroll_)
        updateCurrentSection();

    //the user scrolls away from a jump target which is still being built
    if (dy != 0 && !programmatic_scroll_)
    {
        scroll_up_ = dy > 0;
        pending_target_.reset();
    }

    if (dy != 0)
        build_timer_.start();
}

/**
 * Content widgets change their size later (on-demand table loaded, figure rendered), measure again.
 * A vertical wheel over an inner scroll area which cannot scroll further scrolls the document. Qt only
 * propagates spontaneous wheel events which the inner area ignored, so it is forwarded here.
 */
bool ReportDocumentView::eventFilter(QObject* watched, QEvent* event)
{
    if (event->type() == QEvent::LayoutRequest && !relayout_timer_.isActive())
        relayout_timer_.start();

    if (event->type() == QEvent::Wheel)
    {
        auto area = qobject_cast<QAbstractScrollArea*>(watched->parent());

        if (area && area->viewport() == watched)
        {
            auto        wheel = static_cast<QWheelEvent*>(event);
            const auto* bar   = area->verticalScrollBar();

            if (innerAreaPassesWheel(bar->value(), bar->minimum(), bar->maximum(),
                                     wheel->angleDelta().x(), wheel->angleDelta().y()))
            {
                wheelEvent(wheel);
                return true;
            }
        }
    }

    return QAbstractScrollArea::eventFilter(watched, event);
}

/**
 * True if a wheel step with the given angle delta passes from an inner scroll area to the document:
 * a vertical step and the inner vertical scroll bar at its limit in that direction, or without range.
 */
bool ReportDocumentView::innerAreaPassesWheel(int value, int minimum, int maximum, int delta_x, int delta_y)
{
    if (delta_y == 0 || std::abs(delta_x) > std::abs(delta_y))
        return false;

    if (minimum >= maximum)
        return true;

    //a positive delta scrolls up
    return delta_y > 0 ? value <= minimum : value >= maximum;
}

/**
 * The current section is the one at the top of the viewport.
 */
void ReportDocumentView::updateCurrentSection()
{
    if (layout_.empty())
    {
        setCurrentSection(nullptr);
        return;
    }

    setCurrentSection(layout_.entry(layout_.indexAt(verticalScrollBar()->value())).section);
}

/**
 */
void ReportDocumentView::setCurrentSection(Section* section)
{
    if (section == current_section_)
        return;

    current_section_ = section;

    emit currentSectionChangedSignal(section);
}

}
