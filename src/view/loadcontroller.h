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

#include <QObject>

#include <memory>
#include <string>
#include <vector>

class COMPASS;
class LoadOperation;
class QProgressDialog;

/**
 * Owns the display-load UX for a view load: the modal progress dialog, the wait cursor,
 * and the two-phase progress (load 0..50, view 50..100). A ViewManager collaborator -
 * ViewManager drives it at its dispatch points (begin on op-start, contentArrived per
 * arrival, the view-phase pair during the view loop, end on finish).
 *
 * begin() is idempotent: it ends any prior cycle first, so a superseded load can never
 * leak a dialog or unbalance the cursor. It is driven from the operation's startedSignal
 * (after the engine's single-op wait), so the dialog/cursor are only created once the
 * load is actually the running one.
 *
 * The cycle is also tied to the operation itself: begin() subscribes to its finishedSignal
 * and closes the UX when the op ends as Cancelled. A cancelled op gets no view phase - and
 * an abandoned one (its source swapped away, e.g. resuming live over a paused load) never
 * reaches ViewManager's end() at all, since its signals are disconnected by then.
 */
class LoadController : public QObject
{
    Q_OBJECT

public:
    LoadController(COMPASS& compass, QObject* parent = nullptr);
    virtual ~LoadController();

    // op has started running (post-wait): show the dialog (sized from the op's target set)
    // + wait cursor, and subscribe to the op's dataChangedSignal to advance the load phase.
    // No dialog when the op suppresses status or has nothing to load.
    void begin(const LoadOperation& op);

    void beginViewPhase(unsigned int num_views); // view loop starting: switch to 50..100
    // one view FINISHED updating (not merely dispatched - a view processing
    // asynchronously reports here once its work is committed)
    void advanceViewPhase();
    // load finished: close dialog + cursor. drain pumps the event loop before closing, so a
    // deferred view redraw runs while the dialog is still up (the normal completion path);
    // pass false when ending from inside another emit, where pumping would re-enter.
    void end(bool drain = true);

private slots:
    void canceledSlot();                    // dialog cancel -> cancel the running load
    // driven directly off the op's dataChangedSignal (un-guarded, so progress is immune to
    // ViewManager's re-entrancy deferral): advance the load phase per non-empty content
    void opDataChangedSlot(const std::vector<std::string>& names, bool reset, bool last);
    // op reached a terminal state: end the UX if it was cancelled (no view phase follows)
    void opFinishedSlot();

private:
    COMPASS& compass_;

    std::unique_ptr<QProgressDialog> dialog_;
    double       value_       {0.0};   // accumulator on a 0..100 scale
    unsigned int load_total_  {0};
    unsigned int view_total_  {0};
    bool         cursor_active_{false};

    // re-entrancy guards for the view-phase progress: QProgressDialog::setValue pumps
    // events for a modal dialog, so a view completion can re-enter advanceViewPhase and
    // end the cycle from inside it (see advanceViewPhase / end)
    bool         in_advance_   {false};
    bool         end_deferred_ {false};

    const LoadOperation*    op_ {nullptr}; // the running op (non-owning, valid for the cycle)
    QMetaObject::Connection op_conn_;      // op.dataChangedSignal -> opDataChangedSlot
    QMetaObject::Connection op_fin_conn_;  // op.finishedSignal    -> opFinishedSlot
};
