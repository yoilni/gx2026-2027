param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$outputDir = Join-Path $projectRoot 'cmake-build-stm32/board_io_host'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
$hardware = Join-Path $projectRoot 'User/Hardware'
$executable = Join-Path $outputDir 'board_io_cases.exe'
& $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' `
    '-I' (Join-Path $PSScriptRoot 'host/board_io_hal') `
    '-I' (Join-Path $hardware 'Indicator/inc') `
    '-I' (Join-Path $hardware 'Input/inc') `
    '-I' (Join-Path $hardware 'Serial/inc') `
    (Join-Path $hardware 'Indicator/src/board_led.c') `
    (Join-Path $hardware 'Input/src/switch_input.c') `
    (Join-Path $hardware 'Serial/src/uart_port.c') `
    (Join-Path $PSScriptRoot 'host/board_io_cases.c') '-o' $executable
if ($LASTEXITCODE -ne 0) { throw 'Board IO host build failed' }
& $executable
if ($LASTEXITCODE -ne 0) { throw 'Board IO checks failed' }
