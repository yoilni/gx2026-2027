param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'cmake-build-stm32/imu_selection_host'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$executable = Join-Path $outputDir 'imu_selection_cases.exe'
& $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' `
    '-I' (Join-Path $projectRoot 'tests/host/hwt101_hal') `
    '-I' (Join-Path $projectRoot 'HWT101') `
    '-I' (Join-Path $projectRoot 'JY901S') `
    '-I' (Join-Path $projectRoot 'Task') `
    '-I' (Join-Path $projectRoot 'Debug') `
    (Join-Path $projectRoot 'tests/host/imu_selection_cases.c') '-o' $executable
if ($LASTEXITCODE -ne 0) { throw 'IMU selection host build failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'IMU selection checks failed' }
