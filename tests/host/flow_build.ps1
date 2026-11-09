param([string]$ProjectRoot)

# Compile the real flow modules against the host fixtures, not copies of
# their state machines. Each module uses its own inc/ and src/ directory.
$flowBuildArgs = @(
  foreach ($flowModule in Get-ChildItem -LiteralPath (Join-Path $ProjectRoot 'User/Flows') -Directory) {
    '-I'
    (Join-Path $flowModule.FullName 'inc')
    Get-ChildItem -LiteralPath (Join-Path $flowModule.FullName 'src') -Filter '*.c' -File |
      Select-Object -ExpandProperty FullName
  }
)
