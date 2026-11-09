param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'cmake-build-stm32/motion_layer_host'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$motionRoot = Join-Path $projectRoot 'User/Motion'
$executable = Join-Path $outputDir 'motion_layer_cases.exe'
& $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' `
    '-I' (Join-Path $PSScriptRoot 'host/motion_hal') `
    '-I' (Join-Path $PSScriptRoot 'host') `
    '-I' (Join-Path $projectRoot 'Task') `
    '-I' (Join-Path $projectRoot 'User/Hardware/Motor/inc') `
    '-I' (Join-Path $motionRoot 'WheelSpeed/inc') `
    '-I' (Join-Path $motionRoot 'Chassis/inc') `
    '-I' (Join-Path $motionRoot 'Vision/inc') `
    (Join-Path $motionRoot 'Chassis/src/chassis_motion.c') `
    (Join-Path $motionRoot 'Chassis/src/chassis_actions.c') `
    (Join-Path $motionRoot 'Chassis/src/heading_control.c') `
    (Join-Path $motionRoot 'Vision/src/vision_motion.c') `
    (Join-Path $PSScriptRoot 'host/motion_layer_cases.c') '-o' $executable
if ($LASTEXITCODE -ne 0) { throw 'Motion layer host build failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Motion layer checks failed' }
