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

class QObject;
class QEvent;

// Crash breadcrumbs: a small lock-free ring buffer of recently dispatched
// Qt events (receiver pointer, class name, object name, event type, time).
// Client::notify records into it on every dispatched event of selected
// types; the fatal-signal handler dumps it into the crash log. Purpose:
// when a crash happens while an event is delivered to an already-destroyed
// QObject (dangling receiver), the receiver register value from the crash
// context can be matched against the recorded pointers to identify what
// the freed object was.
namespace crash_breadcrumbs
{

// record an event delivered to a receiver; cheap, lock-free, called from
// Client::notify. Only selected event types are stored (no mouse-move /
// paint / timer flood), so the buffer spans a useful time range.
void record(QObject* receiver, QEvent* event);

// write all stored entries to the given fd; async-signal-safe (no
// allocation, no locks, write() only)
void dumpTo(int fd);

// async-signal-safe write helpers, shared with the signal handler
void writeStr(int fd, const char* s);
void writeDec(int fd, unsigned long long value);
void writeHex(int fd, unsigned long long value);

}  // namespace crash_breadcrumbs
