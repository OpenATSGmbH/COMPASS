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

#include "compass_result_export.h"

#include "report/reportdocumentlayout.h"

#include <QAbstractScrollArea>
#include <QPointer>
#include <QTimer>

#include <atomic>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <vector>

namespace ResultReport
{

class Section;
class SectionContent;

/**
 * Shows all sections of a report one after the other as one scrollable document.
 * Only the sections near the visible area have widgets, the others have estimated heights.
 */
class COMPASS_RESULT_API ReportDocumentView : public QAbstractScrollArea
{
    Q_OBJECT

signals:
    void currentSectionChangedSignal(Section* section);

public:
    explicit ReportDocumentView(QWidget* parent = nullptr);
    virtual ~ReportDocumentView();

    void setRootSection(Section* root, const ReportDocumentLayout::SectionSet& collapsed);
    void setCollapsedSections(const ReportDocumentLayout::SectionSet& collapsed);
    void clear();

    void scrollToSection(Section* section,
                         int offset_in_section,
                         bool preload_ondemand_contents);

    Section* currentSection() const;
    int offsetInCurrentSection() const;

    static bool innerAreaPassesWheel(int value, int minimum, int maximum, int delta_x, int delta_y);

protected:
    void resizeEvent(QResizeEvent* event) override;
    void scrollContentsBy(int dx, int dy) override;
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    struct BuiltEntry
    {
        QWidget*          widget = nullptr; // child of the viewport, holds the heading and the content widget
        QPointer<QWidget> content;          // owned by the section, null once the section deleted it
    };

    struct Target
    {
        Section* section = nullptr;
        int      offset  = 0;
        bool     preload_ondemand_contents = false;
    };

    struct PreloadItem
    {
        Section*                        section = nullptr;
        size_t                          index   = 0; // content index in the section
        unsigned int                    id      = 0;
        std::shared_ptr<SectionContent> content;     // set by the worker
    };

    // contents of the next sections, loaded by a worker which never writes into a section
    struct Preload
    {
        std::future<std::vector<PreloadItem>> future;
        std::vector<Section*>                 sections;
        std::shared_ptr<std::atomic<bool>>    cancel;
    };

    void buildStep();
    std::pair<size_t, size_t> entryRange(int margin) const;
    std::optional<size_t> nextEntryToBuild() const;
    void applyPendingTarget();
    void buildEntry(size_t index, bool preload_ondemand_contents);
    QWidget* createEntryWidget(const ReportDocumentLayout::Entry& entry, QWidget* content);
    void watchInnerScrollAreas(QWidget* widget);
    void releaseEntry(size_t index);
    void releaseBuiltEntry(Section* section, const BuiltEntry& built);
    void releaseFarEntries();
    void releaseAll();

    void startPreload();
    void pollPreload();
    void finishPreload();
    void discardPreload();

    void relayout(bool width_changed);
    void positionEntries();
    void updateScrollRange();
    void setScrollValue(int value);
    int entryWidth(QWidget* widget) const;

    void updateCurrentSection();
    void setCurrentSection(Section* section);

    ReportDocumentLayout         layout_;
    std::map<size_t, BuiltEntry> built_;

    std::optional<Target>  pending_target_;
    std::optional<Preload> preload_;

    QTimer build_timer_;
    QTimer relayout_timer_;
    QTimer preload_timer_;

    // a content widget may run a nested event loop while it is created (on-demand load of a task result)
    bool         building_   = false;
    unsigned int generation_ = 0;

    bool     scroll_up_           = false; // direction of the last user scroll, the preload follows it
    bool     programmatic_scroll_ = false;
    int      viewport_width_      = -1;
    Section* current_section_     = nullptr;
};

}
