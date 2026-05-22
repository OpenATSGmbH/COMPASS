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

//#define X  1
//#pragma push_macro("X")
//#undef X
//#define X -1
//#pragma pop_macro("X")
//int x [X];

#ifdef emit

#pragma push_macro("emit")
#undef emit

#include <tbb/tbb.h>
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_unordered_map.h>

#pragma pop_macro("emit")

#else

#include <tbb/tbb.h>
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_unordered_map.h>

#endif

