#!/bin/bash
#
# Collects the logs of a test run on the Windows test PC crunch_win into
# ~/tmp/crunch_win_logs/<date_time>:
#   suite logs     C:\compass\*.log
#   test logs      C:\data\test\<dataset>\temp\<module>\*_log.txt, *_client_log.txt,
#                  *_client_log_quit_stacks.txt (gdb stacks of an instance that did not quit)
#   crash stacks   C:\compass\crash\* (crunch_win_crash_gdb.bat)
#
# Usage: crunch_win_collect.sh [dataset]   (default at_20230422)

set -e

HOST=crunch_win
DATASET=${1:-at_20230422}
OUT="$HOME/tmp/crunch_win_logs/$(date +%Y%m%d_%H%M%S)"

#runs a PowerShell command on the test PC
remote() {
    ssh -o BatchMode=yes "$HOST" "$1" 2>&1 | grep -v '^\*\*' || true
}

#copies the files into one staging folder (temp\ without databases), then one tar file
remote "\$s = 'C:\\compass\\collect'; if (Test-Path \$s) { Remove-Item -Recurse -Force \$s }
New-Item -ItemType Directory -Force -Path \"\$s\\suite\",\"\$s\\temp\",\"\$s\\crash\" | Out-Null
Copy-Item 'C:\\compass\\*.log' \"\$s\\suite\"
if (Test-Path 'C:\\compass\\crash') { Copy-Item 'C:\\compass\\crash\\*' \"\$s\\crash\" }
\$t = 'C:\\data\\test\\$DATASET\\temp'
Get-ChildItem \$t -Recurse -Filter '*_log*.txt' | ForEach-Object {
    \$d = Join-Path \"\$s\\temp\" \$_.Directory.FullName.Substring(\$t.Length)
    New-Item -ItemType Directory -Force -Path \$d | Out-Null
    Copy-Item \$_.FullName \$d }
tar -cf C:\\compass\\collect.tar -C \$s .
Remove-Item -Recurse -Force \$s"

mkdir -p "$OUT"
scp -q -o BatchMode=yes "$HOST:C:/compass/collect.tar" "$OUT/collect.tar"
remote "Remove-Item C:\\compass\\collect.tar"
tar -xf "$OUT/collect.tar" -C "$OUT"
rm "$OUT/collect.tar"

echo "collected to $OUT"
ls "$OUT/crash" "$OUT/temp"/*/*_quit_stacks.txt 2>/dev/null || true
