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
    void advanceViewPhase();                // one view finished
    void end();                             // load finished/abandoned: close dialog + cursor

private slots:
    void canceledSlot();                    // dialog cancel -> cancel the running load
    // driven directly off the op's dataChangedSignal (un-guarded, so progress is immune to
    // ViewManager's re-entrancy deferral): advance the load phase per non-empty content
    void opDataChangedSlot(const std::vector<std::string>& names, bool reset, bool last);

private:
    COMPASS& compass_;

    std::unique_ptr<QProgressDialog> dialog_;
    double       value_       {0.0};   // accumulator on a 0..100 scale
    unsigned int load_total_  {0};
    unsigned int view_total_  {0};
    bool         cursor_active_{false};

    QMetaObject::Connection op_conn_;  // op.dataChangedSignal -> opDataChangedSlot
};
