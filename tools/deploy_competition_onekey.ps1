param(
  [string]$RobotIp = $(if ($env:ROBOT_IP) { $env:ROBOT_IP } else { "192.168.43.60" }),
  [switch]$Run,
  [switch]$StopExisting,
  [switch]$AllowFixedXyFallback,
  [switch]$ForceFixedTarget
)
$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$argsList = @("$scriptDir/deploy_competition_onekey.py", "--host", $RobotIp)
if ($Run) { $argsList += "--run" }
if ($StopExisting) { $argsList += "--stop-existing" }
if ($AllowFixedXyFallback) { $argsList += "--allow-fixed-xy-fallback" }
if ($ForceFixedTarget) { $argsList += "--force-fixed-target" }
py @argsList
