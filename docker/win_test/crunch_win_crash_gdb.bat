@echo off
rem Crash handler on the Windows test PC: Windows starts it (AeDebug registry key, set by
rem crunch_win_deploy.sh gdb) when a process ends with an unhandled exception. For COMPASS
rem processes, gdb attaches and writes the stacks of all threads to C:\compass\crash, then
rem ends the process. Other processes are not handled (default Windows behavior).
rem AeDebug needs Windows Error Reporting enabled (Disabled 0, DontShowUI 1 for no dialog).
rem
rem Usage (from AeDebug): crunch_win_crash_gdb.bat <pid> <event handle>
tasklist /FI "PID eq %1" /NH | findstr /I "compass_" > nul || exit /b 0
if not exist C:\compass\crash mkdir C:\compass\crash
rem time limit for gdb: a process that ends during the attach (e.g. abort) can block gdb,
rem and the crashed process stays until gdb ends. The watchdog ends gdb instances older than 110 s.
start "" /b powershell -NoProfile -Command "Start-Sleep 120; Get-Process gdb -ErrorAction SilentlyContinue | Where-Object { $_.StartTime -lt (Get-Date).AddSeconds(-110) } | Stop-Process -Force"
rem gdb sets the event after the attach (crunch_win_set_event.exe), the process continues and
rem the exception stops it in gdb at the crash location
C:\compass\gdb\bin\gdb.exe -batch -nx -p %1 -ex "set pagination off" -ex "shell C:\compass\gdb\crunch_win_set_event.exe %2" -ex "continue" -ex "info threads" -ex "thread apply all bt full" -ex "kill" > C:\compass\crash\crash_pid%1.txt 2>&1
