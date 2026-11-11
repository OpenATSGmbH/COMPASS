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
 *
 * You should have received a copy of the GNU General Public License
 * along with COMPASS. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Sets the AeDebug event (inherited handle, given as decimal number) on the Windows test PC.
 * crunch_win_crash_gdb.bat calls it from gdb after the attach: the crashed process then
 * continues, and the exception goes to gdb at the crash location.
 *
 * Build: x86_64-w64-mingw32-gcc -O2 -static -o crunch_win_set_event.exe crunch_win_set_event.c
 */

#include <windows.h>
#include <stdlib.h>

int main(int argc, char** argv)
{
    if (argc < 2)
        return 1;

    HANDLE event = (HANDLE)(INT_PTR)_strtoi64(argv[1], NULL, 10);

    return SetEvent(event) ? 0 : 2;
}
