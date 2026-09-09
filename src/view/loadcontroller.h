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
#include <QPointer>

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

    // paints a dialog synchronously the moment it gets shown (see the filter install
    // sites): both dialogs appear via deferred shows, and the first paint event can
    // starve behind long queued work - the window then maps black for a second
    bool eventFilter(QObject* watched, QEvent* event) override;

    // op has started running (post-wait): show the dialog (sized from the op's target set)
    // + wait cursor, and subscribe to the op's dataChangedSignal to advance the load phase.
    // No dialog when the op suppresses status or has nothing to load.
    void begin(const LoadOperation& op);

    void beginViewPhase(unsigned int num_views); // view loop starting: switch to 50..100
    // one view FINISHED updating (not merely dispatched - a view processing
    // asynchronously reports here once its work is committed)
    void advanceViewPhase();

    // busy state for asynchronous view processing OUTSIDE a load cycle (interactive
    // redraws and reloads): an application-modal busy dialog, so the user cannot
    // mutate view state or close views while workers still read them - the same
    // protection the load dialog provides during a load. No-ops while a load cycle
    // is active (its own dialog covers that).
    void beginViewProcessing();
    void endViewProcessing();

    // suppress both progress dialogs (wait cursor stays): used during report export,
    // where each figure triggers a load + view processing cycle - the modal export
    // dialog already blocks user interaction, and the popping dialogs steal OS focus
    // from other applications
    void setDialogsSuppressed(bool suppressed) { dialogs_suppressed_ = suppressed; }
    // load finished: close dialog + cursor. drain pumps the event loop before closing, so a
    // deferred view redraw runs while the dialog is still up (the normal completion path);
    // pass false when ending from inside another emit, where pumping would re-enter.
    void end(bool drain = true);

private:
    // the deferred tail of an end() that arrived while Qt was executing inside
    // dialog_->setValue(): deletes the dialogs parked in stale_dialogs_. Called by the
    // setValue sites right after setValue returned, so Qt has left the dialog for good
    // before its DeferredDelete can be delivered.
    void deleteStaleDialogs();

private slots:
    void canceledSlot();                    // dialog cancel -> cancel the running load
    // driven directly off the op's dataChangedSignal (un-guarded, so progress is immune to
    // ViewManager's re-entrancy deferral): advance the load phase per non-empty content
    void opDataChangedSlot(const std::vector<std::string>& names, bool reset, bool last);
    // op reached a terminal state: end the UX if it was cancelled (no view phase follows)
    void opFinishedSlot();

private:
    COMPASS& compass_;

    // QPointer, not unique_ptr: the dialog is parented to the active window, so Qt
    // deletes it together with that window (application shutdown, or the window closing
    // under a load). An owning pointer cannot see that and is left dangling, which a
    // late view completion then dereferences - the guarded pointer nulls itself instead.
    // Ownership is still ours in the normal path: end() closes and deletes it.
    QPointer<QProgressDialog> dialog_;
    double       value_       {0.0};   // accumulator on a 0..100 scale
    unsigned int load_total_  {0};
    unsigned int view_total_  {0};
    bool         cursor_active_{false};

    // re-entrancy guard shared by ALL dialog_->setValue call sites (opDataChangedSlot,
    // beginViewPhase, advanceViewPhase): QProgressDialog::setValue pumps events for a
    // modal dialog, so a load/view completion can re-enter any of them from inside it.
    // The guard ensures at most one setValue pump is ever on the stack; nested calls
    // only accumulate value_, the single active pump paints the final value. It also
    // marks "Qt is currently executing inside dialog_": an end() reached through the
    // pump must NOT deleteLater the dialog - a deleteLater posted from nested event
    // delivery carries an inflated scope level, so the pump still running inside
    // setValue delivers the DeferredDelete before setValue returns, and Qt then
    // touches the freed progress bar (crashed in QProgressBar::maximum on 2026-08-17
    // and again on 2026-08-31). Such an end() closes the dialog and parks it in
    // stale_dialogs_; the setValue site deletes it via deleteStaleDialogs() once
    // setValue has returned.
    bool         in_set_value_ {false};

    // dialogs whose end() arrived while Qt was executing inside their setValue (see
    // in_set_value_): closed immediately there, deleted via deleteStaleDialogs() at
    // the setValue call sites. QPointer for the same parenting reason as dialog_.
    std::vector<QPointer<QProgressDialog>> stale_dialogs_;

    const LoadOperation*    op_ {nullptr}; // the running op (non-owning, valid for the cycle)
    QMetaObject::Connection op_conn_;      // op.dataChangedSignal -> opDataChangedSlot
    QMetaObject::Connection op_fin_conn_;  // op.finishedSignal    -> opFinishedSlot

    // the interactive view-processing busy dialog (see beginViewProcessing); QPointer
    // for the same parenting reason as dialog_
    QPointer<QProgressDialog> busy_dialog_;

    bool dialogs_suppressed_ {false}; // see setDialogsSuppressed()
};
