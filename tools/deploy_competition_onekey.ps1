param(
  [string]$RobotIp = $(if ($env:ROBOT_IP) { $env:ROBOT_IP } else { "192.168.43.60" }),
  [switch]$Run,
  [switch]$StopExisting
)
$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$argsList = @("$scriptDir/deploy_competition_onekey.py", "--host", $RobotIp)
if ($Run) { $argsList += "--run" }
if ($StopExisting) { $argsList += "--stop-existing" }
py @argsList
