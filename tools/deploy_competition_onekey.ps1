param(
  [string]$RobotIp = $env:ROBOT_IP,
  [switch]$Run,
  [switch]$StopExisting,
  [switch]$AllowFixedXyFallback,
  [switch]$ForceFixedTarget,
  [switch]$StrictResultGates
)
$ErrorActionPreference = "Stop"
if ([string]::IsNullOrWhiteSpace($RobotIp)) {
  throw "Set -RobotIp or ROBOT_IP before deploying"
}
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$argsList = @("$scriptDir/deploy_competition_onekey.py", "--host", $RobotIp)
if ($Run) { $argsList += "--run" }
if ($StopExisting) { $argsList += "--stop-existing" }
if ($AllowFixedXyFallback) { $argsList += "--allow-fixed-xy-fallback" }
if ($ForceFixedTarget) { $argsList += "--force-fixed-target" }
if ($StrictResultGates) { $argsList += "--strict-result-gates" }
py @argsList
