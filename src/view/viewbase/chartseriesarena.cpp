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

#include "chartseriesarena.h"

#include "logger.h"

#include <QLineSeries>
#include <QScatterSeries>

#include <algorithm>
#include <cstddef>
#include <mutex>
#include <new>
#include <vector>

namespace
{
    enum class Region { Data, Selection };

    void* arenaAllocate(size_t size, Region region);
    void  arenaDeallocate(void* p);

    /**
     * A Qt Charts series whose memory comes from the arena. No own members, only the
     * allocation functions, which delete through the virtual destructor of QObject reaches.
     */
    template <class Base, Region R>
    class ArenaSeries final : public Base
    {
    public:
        static void* operator new(size_t size) { return arenaAllocate(size, R); }
        static void  operator delete(void* p)  { arenaDeallocate(p); }
    };

    using DataScatterSeries   = ArenaSeries<QtCharts::QScatterSeries, Region::Data>;
    using DataLineSeries      = ArenaSeries<QtCharts::QLineSeries,    Region::Data>;
    using SelectionLineSeries = ArenaSeries<QtCharts::QLineSeries,    Region::Selection>;

    constexpr size_t roundUp(size_t value, size_t alignment)
    {
        return (value + alignment - 1) / alignment * alignment;
    }

    /// one slot holds any of the series types
    constexpr size_t SlotSize = roundUp(std::max({ sizeof(DataScatterSeries),
                                                   sizeof(DataLineSeries),
                                                   sizeof(SelectionLineSeries) }),
                                        alignof(std::max_align_t));

    /// a scatter plot has one scatter series per data series, and one line series with connection lines
    const size_t NumDataSlots      = 16384;
    const size_t NumSelectionSlots = 1024;

    /**
     * The memory block: data slots first, the selection slots above them.
     */
    class Arena
    {
    public:
        static Arena& instance()
        {
            //never destroyed: the series of a chart may be deleted during the static destruction
            static Arena* arena = new Arena;
            return *arena;
        }

        void* allocate(size_t size, Region region)
        {
            std::lock_guard<std::mutex> lock(mutex_);

            auto& free_slots = region == Region::Data ? free_data_ : free_selection_;

            if (size > SlotSize || free_slots.empty())
            {
                if (!warned_)
                {
                    logwrn << "no free slot for a "
                           << (region == Region::Data ? "data" : "selection")
                           << " series, the selection may be drawn below the data";
                    warned_ = true;
                }

                return ::operator new(size);
            }

            const size_t slot = free_slots.back();
            free_slots.pop_back();

            return block_ + slot * SlotSize;
        }

        void deallocate(void* p)
        {
            if (!p)
                return;

            std::lock_guard<std::mutex> lock(mutex_);

            if (!inBlock(p))
            {
                ::operator delete(p);
                return;
            }

            const size_t slot = ((unsigned char*)p - block_) / SlotSize;

            if (slot < NumDataSlots)
                free_data_.push_back(slot);
            else
                free_selection_.push_back(slot);
        }

        bool inRegion(const void* p, Region region) const
        {
            if (!inBlock(p))
                return false;

            const size_t slot = ((const unsigned char*)p - block_) / SlotSize;

            return region == Region::Data ? slot < NumDataSlots : slot >= NumDataSlots;
        }

        size_t numUsed(Region region) const
        {
            std::lock_guard<std::mutex> lock(mutex_);

            return region == Region::Data ? NumDataSlots - free_data_.size()
                                          : NumSelectionSlots - free_selection_.size();
        }

    private:
        Arena()
        :   block_((unsigned char*)::operator new(SlotSize * (NumDataSlots + NumSelectionSlots)))
        {
            //descending, so the lowest free slot is taken first
            free_data_.reserve(NumDataSlots);
            for (size_t i = NumDataSlots; i > 0; --i)
                free_data_.push_back(i - 1);

            free_selection_.reserve(NumSelectionSlots);
            for (size_t i = NumDataSlots + NumSelectionSlots; i > NumDataSlots; --i)
                free_selection_.push_back(i - 1);
        }

        bool inBlock(const void* p) const
        {
            auto addr = (const unsigned char*)p;
            return addr >= block_ && addr < block_ + SlotSize * (NumDataSlots + NumSelectionSlots);
        }

        unsigned char*      block_ = nullptr;
        std::vector<size_t> free_data_;
        std::vector<size_t> free_selection_;
        mutable std::mutex  mutex_;
        bool                warned_ = false;
    };

    void* arenaAllocate(size_t size, Region region)
    {
        return Arena::instance().allocate(size, region);
    }

    void arenaDeallocate(void* p)
    {
        Arena::instance().deallocate(p);
    }
}

namespace chart_series_arena
{

/**
 */
QtCharts::QScatterSeries* createDataScatterSeries()
{
    return new DataScatterSeries;
}

/**
 */
QtCharts::QLineSeries* createDataLineSeries()
{
    return new DataLineSeries;
}

/**
 */
QtCharts::QLineSeries* createSelectionLineSeries()
{
    return new SelectionLineSeries;
}

/**
 */
bool inDataRegion(const void* series)
{
    return Arena::instance().inRegion(series, Region::Data);
}

/**
 */
bool inSelectionRegion(const void* series)
{
    return Arena::instance().inRegion(series, Region::Selection);
}

/**
 */
size_t numDataSeries()
{
    return Arena::instance().numUsed(Region::Data);
}

/**
 */
size_t numSelectionSeries()
{
    return Arena::instance().numUsed(Region::Selection);
}

}
