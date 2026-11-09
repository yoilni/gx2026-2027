# The legacy focused host cases compile selected real Mission functions beside
# HAL/RTOS stubs. Flatten storage only in that test fixture; the firmware links
# the normal translation units and does not use this projection.
function Get-MissionHostFixture([string]$ProjectRoot) {
  $root = Join-Path $ProjectRoot 'User/Mission'
  $header = Get-Content -Raw -LiteralPath (Join-Path $root 'State/inc/mission_internal.h')
  $files = @(Get-ChildItem -LiteralPath $root -Directory |
    ForEach-Object { Get-ChildItem -LiteralPath (Join-Path $_.FullName 'src') -Filter 'mission_*.c' -File } |
    Sort-Object FullName)
  $units = @($files | ForEach-Object { Get-Content -Raw -LiteralPath $_.FullName })
  $bodies = $units -join "`n"
  $state = [regex]::Match($header, '(?ms)^typedef struct\r?\n\{\r?\n(.*?)^\} MissionContext;')
  $initializers = [regex]::Match($bodies, '(?ms)^MissionContext mission_ctx = \{\r?\n(.*?)^\};')
  if (!$state.Success -or !$initializers.Success) { throw 'Missing MissionContext definition' }

  $initial = @{}
  foreach ($init in [regex]::Matches($initializers.Groups[1].Value, '(?m)^  \.(\w+) = (.*),\r?$')) {
    $initial[$init.Groups[1].Value] = $init.Groups[2].Value
  }
  $enums = @([regex]::Matches($header + "`n" + $bodies, '(?ms)^typedef enum\r?\n\{.*?^\} \w+;') |
    ForEach-Object { $_.Value })
  $shared = @(foreach ($field in [regex]::Matches($state.Groups[1].Value, '(?m)^  (\w+) (\w+);\r?$')) {
    $name = $field.Groups[2].Value
    $value = if ($initial.ContainsKey($name)) { " = $($initial[$name])" } else { '' }
    "static $($field.Groups[1].Value) $name$value;"
  })
  $locals = @([regex]::Matches($bodies, '(?m)^static \w+ \w+(?: = [^\r\n;]+)?;\r?$') |
    ForEach-Object { $_.Value })
  $eventDefines = [regex]::Match($header, '(?ms)^#define MISSION_EVENT_START\b.*?(?=^\r?$)').Value
  if (!$eventDefines -or $shared.Count -eq 0) { throw 'Missing Mission event or state definitions' }

  # Focused host fixtures use the pre-split names for their assertions/stubs.
  $source = $bodies -creplace '\bmission_ctx\.(\w+)', '$1'
  $source = $source -creplace '(?m)^(?!static\b)(\w[\w *]*\bMissionTask_\w+\()', 'static $1'
  @{
    Source = $source
    Globals = ($enums + $shared + $locals) -join "`n"
    EventDefines = $eventDefines
  }
}
