param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'cmake-build-stm32/hwt101_host'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$executable = Join-Path $outputDir 'hwt101_cases.exe'
& $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' `
    '-I' (Join-Path $projectRoot 'tests/host/hwt101_hal') `
    '-I' (Join-Path $projectRoot 'User/Hardware/Imu/inc') `
    '-I' (Join-Path $projectRoot 'User/Hardware/Imu/src') `
    '-I' (Join-Path $projectRoot 'User/Hardware/Serial/inc') `
    '-I' (Join-Path $projectRoot 'Task') `
    '-I' (Join-Path $projectRoot 'Debug') `
    (Join-Path $projectRoot 'tests/host/hwt101_cases.c') `
    (Join-Path $projectRoot 'User/Runtime/Diagnostics/src/imu_diagnostics_adapter.c') '-o' $executable
if ($LASTEXITCODE -ne 0) { throw 'HWT101 host build failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'HWT101 checks failed' }
