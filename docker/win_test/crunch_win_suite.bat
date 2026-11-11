@echo off
rem Runs the Python integration test suite on crunch_win against the deployed Windows package.
rem Start it in the desktop session with crunch_win_run.ps1. Extra arguments (e.g. --tags=db)
rem are passed on to test_suite.py.
rem   dataset:  C:\data\test\at_20230422 (manifest at_20230422_05h.json)
rem   results:  C:\data\test\at_20230422\results
set PYTHONUTF8=1
set PYTHONWARNINGS=ignore
"C:\Program Files\Python314\python.exe" -u C:\compass\py\test_infra\test_suite.py --no-prompt ^
  --binary=C:\compass\COMPASS_1.1.0_win64\compass_client.exe ^
  --data=C:\data\test\at_20230422 --manifest=C:\data\test\at_20230422\at_20230422_05h.json ^
  --mem-interval 0 %*
echo suite exit %ERRORLEVEL%
