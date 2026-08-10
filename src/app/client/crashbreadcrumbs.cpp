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

#include "crashbreadcrumbs.h"

#include <QDateTime>
#include <QEvent>
#include <QObject>

#include <atomic>
#include <cstdint>
#include <cstring>

#include <unistd.h>

namespace crash_breadcrumbs
{

namespace
{

constexpr size_t NUM_ENTRIES = 128;
constexpr size_t CLASS_NAME_LEN = 48;
constexpr size_t OBJECT_NAME_LEN = 32;

// One recorded event dispatch. seq doubles as validity marker: 0 means
// empty or currently being written; entries are published by storing the
// nonzero sequence number last (release), and the dumping signal handler
// reads it first (acquire), so half-written entries are never dumped.
// The char arrays are re-zeroed before each write and strncpy never
// touches the last byte, so they stay NUL-terminated even if a concurrent
// writer wraps onto the same slot.
struct Entry
{
    std::atomic<unsigned long long> seq{0};
    unsigned long long ms_since_epoch{0};
    const void* receiver{nullptr};
    int event_type{0};
    char class_name[CLASS_NAME_LEN]{};
    char object_name[OBJECT_NAME_LEN]{};
};

Entry entries[NUM_ENTRIES];
std::atomic<unsigned long long> next_seq{1};

// interaction / lifetime events only - recording every event (paint, timer,
// mouse move) would wrap the buffer within milliseconds and destroy the
// usable history
bool shouldRecord(int type)
{
    switch (type)
    {
        case QEvent::MouseButtonPress:
        case QEvent::MouseButtonRelease:
        case QEvent::MouseButtonDblClick:
        case QEvent::Wheel:
        case QEvent::KeyPress:
        case QEvent::Enter:
        case QEvent::Leave:
        case QEvent::FocusIn:
        case QEvent::FocusOut:
        case QEvent::ContextMenu:
        case QEvent::Show:
        case QEvent::Hide:
        case QEvent::Close:
        case QEvent::DeferredDelete:
            return true;
        default:
            return false;
    }
}

}  // namespace

void record(QObject* receiver, QEvent* event)
{
    if (!receiver || !event || !shouldRecord(event->type()))
        return;

    unsigned long long seq = next_seq.fetch_add(1, std::memory_order_relaxed);
    Entry& e = entries[seq % NUM_ENTRIES];

    e.seq.store(0, std::memory_order_release);  // mark in-progress

    e.ms_since_epoch = static_cast<unsigned long long>(QDateTime::currentMSecsSinceEpoch());
    e.receiver = receiver;
    e.event_type = static_cast<int>(event->type());

    std::memset(e.class_name, 0, sizeof(e.class_name));
    std::strncpy(e.class_name, receiver->metaObject()->className(), sizeof(e.class_name) - 1);

    std::memset(e.object_name, 0, sizeof(e.object_name));
    const QString object_name = receiver->objectName();
    if (!object_name.isEmpty())
        std::strncpy(e.object_name, object_name.toUtf8().constData(), sizeof(e.object_name) - 1);

    e.seq.store(seq, std::memory_order_release);  // publish
}

void writeStr(int fd, const char* s)
{
    size_t len = 0;
    while (s[len])
        ++len;

    ssize_t unused = write(fd, s, len);
    (void)unused;
}

void writeDec(int fd, unsigned long long value)
{
    char buf[24];
    int pos = sizeof(buf);
    do
    {
        buf[--pos] = static_cast<char>('0' + (value % 10));
        value /= 10;
    } while (value > 0);

    ssize_t unused = write(fd, buf + pos, sizeof(buf) - pos);
    (void)unused;
}

void writeHex(int fd, unsigned long long value)
{
    const char* hexmap = "0123456789abcdef";

    char buf[18];
    int pos = sizeof(buf);
    do
    {
        buf[--pos] = hexmap[value & 0xF];
        value >>= 4;
    } while (value > 0);
    buf[--pos] = 'x';
    buf[--pos] = '0';

    ssize_t unused = write(fd, buf + pos, sizeof(buf) - pos);
    (void)unused;
}

void dumpTo(int fd)
{
    writeStr(fd, "\nRecent event breadcrumbs (seq time_ms receiver event_type class object):\n");

    // slots are dumped in storage order; readers can sort by seq. Skipping
    // unpublished slots (seq 0) avoids torn entries.
    for (size_t i = 0; i < NUM_ENTRIES; ++i)
    {
        const Entry& e = entries[i];

        unsigned long long seq = e.seq.load(std::memory_order_acquire);
        if (!seq)
            continue;

        writeDec(fd, seq);
        writeStr(fd, " ");
        writeDec(fd, e.ms_since_epoch);
        writeStr(fd, " ");
        writeHex(fd, reinterpret_cast<uintptr_t>(e.receiver));
        writeStr(fd, " ");
        writeDec(fd, static_cast<unsigned long long>(e.event_type));
        writeStr(fd, " ");
        writeStr(fd, e.class_name[0] ? e.class_name : "-");
        writeStr(fd, " ");
        writeStr(fd, e.object_name[0] ? e.object_name : "-");
        writeStr(fd, "\n");
    }
}

}  // namespace crash_breadcrumbs
