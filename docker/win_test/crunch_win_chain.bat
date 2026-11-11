@echo off
rem Full test chain on crunch_win, as run_ci_tests.sh on crunch: the whole suite once per test
rem dataset, each with --reset. The datasets must be in C:\data\test (crunch_win_deploy.sh data
rem <dataset>). Start it in the desktop session with crunch_win_run.ps1.
rem Usage: crunch_win_chain.bat [dataset ...]   (no arguments: all datasets)
rem   log per dataset:     C:\compass\chain_<dataset>.log
rem   results per dataset: C:\data\test\<dataset>\results
if "%~1"=="" (
    call :run at_20230422
    call :run loww_20260609
    call :run skeyes_20251203
    call :run eddf_20251117
) else (
    for %%d in (%*) do call :run %%d
)
echo chain done
exit /b 0

:run
set DS=%1
if "%DS%"=="at_hello_world" set MF=manifest.json
if "%DS%"=="at_20230422" set MF=at_20230422_05h.json
if "%DS%"=="loww_20260609" set MF=loww_20260609_4h.json
if "%DS%"=="skeyes_20251203" set MF=skeyes_20251203.json
if "%DS%"=="eddf_20251117" set MF=eddf_20251117.json
echo === dataset %DS% start %DATE% %TIME%
call C:\compass\crunch_win_suite.bat --reset --data=C:\data\test\%DS% --manifest=C:\data\test\%DS%\%MF% > C:\compass\chain_%DS%.log 2>&1
echo === dataset %DS% done %DATE% %TIME%
findstr /C:"test(s) succeeded" /C:"suite exit" C:\compass\chain_%DS%.log
exit /b 0
