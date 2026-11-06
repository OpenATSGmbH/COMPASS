# Runs a command line in the desktop session of the logged-on user (OpenGL needs it, SSH
# sessions have no desktop and end their child processes at logout). Output goes to the log
# file. Returns at once; poll the log file or Get-ScheduledTask for the state.
#
# Usage: powershell -File crunch_win_run.ps1 -Name <task name> -Command "<cmd line>" -Log <file>
param([string]$Name, [string]$Command, [string]$Log)

$action    = New-ScheduledTaskAction -Execute 'cmd.exe' -Argument "/c $Command > `"$Log`" 2>&1"
$principal = New-ScheduledTaskPrincipal -UserId $env:USERNAME -LogonType Interactive -RunLevel Highest
$settings  = New-ScheduledTaskSettingsSet -ExecutionTimeLimit (New-TimeSpan -Hours 24) -AllowStartIfOnBatteries
Register-ScheduledTask -TaskName $Name -Action $action -Principal $principal -Settings $settings -Force | Out-Null
Start-ScheduledTask -TaskName $Name
"started $Name"
