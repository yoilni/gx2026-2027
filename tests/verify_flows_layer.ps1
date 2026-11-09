param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'cmake-build-stm32/flows_layer_host'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$executable = Join-Path $outputDir 'flows_layer_cases.exe'
. (Join-Path $PSScriptRoot 'host/flow_build.ps1') $projectRoot
$chassis = Join-Path $projectRoot 'User/Motion/Chassis'
& $Compiler -std=c11 -Wall -Wextra -Werror `
  '-I' (Join-Path $PSScriptRoot 'host') '-I' (Join-Path $projectRoot 'User/Config/inc') `
  '-I' (Join-Path $chassis 'inc') (Join-Path $chassis 'src/heading_control.c') `
  (Join-Path $chassis 'src/chassis_actions.c') (Join-Path $PSScriptRoot 'host/flows_layer_cases.c') `
  @flowBuildArgs '-o' $executable
if ($LASTEXITCODE -ne 0) { throw 'Flows layer host build failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Flows layer checks failed' }
