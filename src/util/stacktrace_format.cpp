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

#include "stacktrace_format.h"

#include <backtrace.h>

#include <boost/core/demangle.hpp>
#include <boost/stacktrace/stacktrace.hpp>

#include <cstdio>
#include <sstream>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace stacktrace_format
{

namespace
{
    void ignoreError(void* /*data*/, const char* /*msg*/, int /*errnum*/) {}

    /// libbacktrace state of the process, reads the debug information of all loaded modules on demand
    backtrace_state* state()
    {
        static backtrace_state* st = backtrace_create_state(nullptr, 1, ignoreError, nullptr);
        return st;
    }

    std::string hex(uintptr_t value)
    {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "0x%llx", (unsigned long long)value);
        return buf;
    }

    std::string baseName(const std::string& path)
    {
        auto pos = path.find_last_of("/\\");
        return pos == std::string::npos ? path : path.substr(pos + 1);
    }

    struct PcInfo
    {
        Frame* frame = nullptr;
        bool   first = true;
    };

    /// function, file and line from the debug information: called once per inline level, innermost first
    int pcInfoCallback(void* data, uintptr_t /*pc*/, const char* file, int line, const char* function)
    {
        auto info = static_cast<PcInfo*>(data);

        Location loc;
        if (function)
            loc.function = boost::core::demangle(function);
        if (file)
        {
            loc.file = file;
            loc.line = line;
        }

        if (info->first)
        {
            info->frame->function = loc.function;
            info->frame->file     = loc.file;
            info->frame->line     = loc.line;
            info->first           = false;
        }
        else
        {
            info->frame->inlined_in.push_back(loc);
        }

        return 0;
    }

#ifdef _WIN32
    /// file name and load address of the module containing the address
    void moduleOf(const void* address, Frame& frame, uintptr_t& base)
    {
        HMODULE module = nullptr;
        if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                                (LPCWSTR)address, &module) || !module)
            return;

        base = (uintptr_t)module;

        wchar_t path[MAX_PATH];
        DWORD len = GetModuleFileNameW(module, path, MAX_PATH);
        if (len == 0)
            return;

        int size = WideCharToMultiByte(CP_UTF8, 0, path, (int)len, nullptr, 0, nullptr, nullptr);
        std::string utf8(size, '\0');
        WideCharToMultiByte(CP_UTF8, 0, path, (int)len, &utf8[0], size, nullptr, nullptr);

        frame.module = baseName(utf8);
    }

    /// name of the closest exported function at or below the address, with the distance to it
    std::string exportedName(uintptr_t base, uintptr_t address)
    {
        auto image = (const unsigned char*)base;
        auto dos   = (const IMAGE_DOS_HEADER*)image;
        if (dos->e_magic != IMAGE_DOS_SIGNATURE)
            return {};

        auto nt = (const IMAGE_NT_HEADERS*)(image + dos->e_lfanew);
        if (nt->Signature != IMAGE_NT_SIGNATURE)
            return {};

        const auto& dir = nt->OptionalHeader.DataDirectory[ IMAGE_DIRECTORY_ENTRY_EXPORT ];
        if (dir.VirtualAddress == 0 || dir.Size == 0)
            return {};

        auto exports   = (const IMAGE_EXPORT_DIRECTORY*)(image + dir.VirtualAddress);
        auto names     = (const DWORD*)(image + exports->AddressOfNames);
        auto ordinals  = (const WORD*) (image + exports->AddressOfNameOrdinals);
        auto functions = (const DWORD*)(image + exports->AddressOfFunctions);

        const uintptr_t rva = address - base;

        DWORD       best_rva  = 0;
        const char* best_name = nullptr;

        for (DWORD i = 0; i < exports->NumberOfNames; ++i)
        {
            DWORD func_rva = functions[ ordinals[ i ] ];

            //forwarders point into the export directory, they are no code
            if (func_rva >= dir.VirtualAddress && func_rva < dir.VirtualAddress + dir.Size)
                continue;

            if (func_rva <= rva && func_rva >= best_rva)
            {
                best_rva  = func_rva;
                best_name = (const char*)(image + names[ i ]);
            }
        }

        if (!best_name)
            return {};

        return boost::core::demangle(best_name) + "+" + hex(rva - best_rva);
    }
#else
    struct SymInfo
    {
        std::string name;
        uintptr_t   value = 0;
    };

    void symInfoCallback(void* data, uintptr_t /*pc*/, const char* symname, uintptr_t symval, uintptr_t /*symsize*/)
    {
        auto info = static_cast<SymInfo*>(data);
        if (symname)
        {
            info->name  = symname;
            info->value = symval;
        }
    }
#endif
}

/**
 */
Frame resolve(const void* address)
{
    Frame frame;

    if (!address)
        return frame;

    //a return address points behind the call, the call is one byte before
    const uintptr_t pc = (uintptr_t)address - 1;

    PcInfo pc_info;
    pc_info.frame = &frame;

    if (auto st = state())
        backtrace_pcinfo(st, pc, pcInfoCallback, ignoreError, &pc_info);

    uintptr_t base = 0;

#ifdef _WIN32
    moduleOf(address, frame, base);

    if (frame.function.empty() && base)
        frame.function = exportedName(base, pc);
#else
    Dl_info info;
    if (dladdr(address, &info))
    {
        if (info.dli_fname)
            frame.module = baseName(info.dli_fname);
        base = (uintptr_t)info.dli_fbase;
    }

    if (frame.function.empty())
    {
        SymInfo sym;
        if (auto st = state())
            backtrace_syminfo(st, pc, symInfoCallback, ignoreError, &sym);

        if (!sym.name.empty())
            frame.function = boost::core::demangle(sym.name.c_str()) + "+" + hex(pc - sym.value);
    }
#endif

    if (base)
        frame.offset = (uintptr_t)address - base;

    return frame;
}

/**
 */
std::string toString(const Frame& frame, const void* address)
{
    std::string line = frame.function.empty() ? std::string("??") : frame.function;

    if (!frame.file.empty())
        line += " at " + frame.file + ":" + std::to_string(frame.line);

    if (!frame.module.empty())
        line += " in " + frame.module + "+" + hex(frame.offset);
    else
        line += " at " + hex((uintptr_t)address);

    for (const auto& loc : frame.inlined_in)
    {
        line += "\n        inlined in " + (loc.function.empty() ? std::string("??") : loc.function);

        if (!loc.file.empty())
            line += " at " + loc.file + ":" + std::to_string(loc.line);
    }

    return line;
}

namespace
{
    /// the frames [first, first + count) of the trace, numbered from 0
    std::string formatFrames(const boost::stacktrace::stacktrace& trace, std::size_t first, std::size_t count)
    {
        std::ostringstream ss;

        for (std::size_t i = first; i < trace.size() && i - first < count; ++i)
        {
            const void* address = trace[ i ].address();
            if (!address)
                continue;

            char idx[16];
            std::snprintf(idx, sizeof(idx), "%2u# ", (unsigned)(i - first));

            ss << idx << toString(resolve(address), address) << "\n";
        }

        return ss.str();
    }
}

/**
 */
std::string toString(const boost::stacktrace::stacktrace& trace)
{
    return formatFrames(trace, 0, trace.size());
}

/**
 */
std::string current(std::size_t max_depth)
{
    //the caller is the frame with the return address of this function. A fixed number of frames to skip
    //does not work: the capture of Boost has one internal frame more on Windows than on Linux.
    const void* caller = __builtin_return_address(0);

    boost::stacktrace::stacktrace trace(0, max_depth + 8);

    std::size_t first = 0;
    for (std::size_t i = 0; i < trace.size(); ++i)
    {
        if (trace[ i ].address() == caller)
        {
            first = i;
            break;
        }
    }

    return formatFrames(trace, first, max_depth);
}

}
