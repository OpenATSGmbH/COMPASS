# Suspends or resumes the running Python test suite (test_suite.py) on potato_win, e.g. to pause a
# long test without ending it. Suspended processes show the status "Suspended" in the Task Manager
# details. The suite reads the output of its COMPASS instance through a pipe, so COMPASS stalls at
# its next log line until the suite is resumed (seen 2026-11-24: no CPU, log and DB stop). A live
# replay inside COMPASS keeps sending in real time, the data of the pause is lost.
#
# Usage: powershell -ExecutionPolicy Bypass -File potato_win_suspend.ps1 -Action suspend|resume|status
param([ValidateSet("suspend", "resume", "status")][string]$Action = "status")

Add-Type -Namespace Native -Name Nt -MemberDefinition @'
[DllImport("ntdll.dll")] public static extern int NtSuspendProcess(IntPtr handle);
[DllImport("ntdll.dll")] public static extern int NtResumeProcess(IntPtr handle);
'@

$procs = Get-CimInstance Win32_Process -Filter "Name = 'python.exe'" |
         Where-Object { $_.CommandLine -like "*test_suite.py*" }

if (-not $procs) { "no running test suite"; exit 1 }

foreach ($cim in $procs) {
    $p = Get-Process -Id $cim.ProcessId
    if ($Action -eq "suspend") {
        $rc = [Native.Nt]::NtSuspendProcess($p.Handle)
    } elseif ($Action -eq "resume") {
        $rc = [Native.Nt]::NtResumeProcess($p.Handle)
    } else {
        $rc = 0
    }
    $waits = ($p.Threads | Where-Object { $_.WaitReason -eq "Suspended" }).Count
    "pid $($p.Id) action $Action rc $rc threads $($p.Threads.Count) suspended $waits"
}
