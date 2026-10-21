# Updates this project to the newest RW-Template release. Your custom folder stays as it is.
#   powershell -ExecutionPolicy Bypass -File tools\update.ps1                    downloads the newest release
#   powershell -ExecutionPolicy Bypass -File tools\update.ps1 RW-Template.zip    uses a zip you already downloaded
# In VS Code you can also run it from Terminal > Run Task > Update RW-Template.
# tools/update.sh does the same on Mac and Linux, and the two should stay in step.
#
# It replaces RW's own files (listed in tools/rw-files.txt) and leaves everything else alone:
# custom/, your .vscode settings, and any files you added. If you changed one of RW's files,
# your copy is saved in rw-backup/ before it's replaced.
#
# After downloading, it runs the new release's copy of this script to do the update, so changes
# to the update steps work right away. Older versions run it as
#   & <new release folder>\tools\update.ps1 -Apply <new release folder> -Project <project folder>
# so keep those parameters working.
param(
  [Parameter(Position = 0)][string]$Zip,
  [string]$Apply,
  [string]$Project
)

$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
$RepoUrl = 'https://github.com/richardbwang/RW-Template'
$DocsUrl = 'https://richardbwang.github.io/RW-Template/getting-started/installation/'
$Sha256 = [Security.Cryptography.SHA256]::Create()

function Stop-Update([string]$Message) {
  Write-Host "Update failed: $Message" -ForegroundColor Red
  exit 1
}

function Test-RwProject([string]$Dir) {
  return ((Test-Path -LiteralPath (Join-Path $Dir 'custom') -PathType Container) -and
          (Test-Path -LiteralPath (Join-Path $Dir 'makefile') -PathType Leaf))
}

# A file in a folder, from a path written with / like the ones in tools/rw-files.txt
function Join-RwPath([string]$Dir, [string]$Path) {
  Join-Path $Dir ($Path.Replace('/', [IO.Path]::DirectorySeparatorChar))
}

function Copy-RwFile([string]$From, [string]$To) {
  New-Item -ItemType Directory -Force -Path (Split-Path -Parent $To) | Out-Null
  Copy-Item -LiteralPath $From -Destination $To -Force
}

# SHA-256 of a file, ignoring CR characters so Windows line endings don't count as a change
function Get-RwHash([string]$File) {
  $bytes = [IO.File]::ReadAllBytes($File)
  if ([Array]::IndexOf($bytes, [byte]13) -ge 0) {
    $kept = New-Object System.IO.MemoryStream
    foreach ($b in $bytes) { if ($b -ne 13) { $kept.WriteByte($b) } }
    $bytes = $kept.ToArray()
  }
  ([BitConverter]::ToString($Sha256.ComputeHash($bytes)) -replace '-', '').ToLowerInvariant()
}

# Paths the updater must never touch
function Test-SafePath([string]$Path) {
  if ($Path -match '^/|\\|^[A-Za-z]:') { return $false }
  if (('/' + $Path + '/') -match '/\.\.?/') { return $false }
  if ($Path -match '^(custom|\.git|rw-backup)/' -or $Path -eq 'tools/rw-files.txt') { return $false }
  return $true
}

# The version and files (path -> hash) in a file list
function Read-RwList([string]$File) {
  $files = New-Object System.Collections.Specialized.OrderedDictionary
  $lines = [IO.File]::ReadAllLines($File)
  $version = ''
  if ($lines.Count -gt 0 -and $lines[0] -cmatch '^# RW-Template ([A-Za-z0-9._-]*)') { $version = $Matches[1] }
  foreach ($line in $lines) {
    if ($line -cmatch '^([0-9a-f]{64})  (.+)$') {
      $hash = $Matches[1]
      $path = $Matches[2]
      if (Test-SafePath $path) { $files[$path] = $hash }
    }
  }
  [pscustomobject]@{ Version = $version; Files = $files }
}

# The lines in a folder's src/ that give a setting its default (RW_DEFAULT)
function Get-RwDefaults([string]$Dir) {
  $src = Join-Path $Dir 'src'
  if (-not (Test-Path -LiteralPath $src -PathType Container)) { return }
  $paths = @(Get-ChildItem -LiteralPath $src -Recurse -File -Filter '*.cpp' | ForEach-Object { $_.FullName })
  [Array]::Sort($paths, [StringComparer]::Ordinal)
  foreach ($path in $paths) {
    foreach ($line in [IO.File]::ReadAllLines($path)) {
      if ($line -cmatch '^RW_DEFAULT\s+[A-Za-z_]\w*\s+([A-Za-z_]\w*)') {
        [pscustomobject]@{ Name = $Matches[1]; Line = ($line -creplace '^RW_DEFAULT\s+', '') }
      }
    }
  }
}

# Removes the folders a removed file left empty, up to the project folder
function Remove-EmptyFolders([string]$Dir, [string]$Root) {
  $rootLength = $Root.TrimEnd('\', '/').Length
  while ($Dir -and $Dir.TrimEnd('\', '/').Length -gt $rootLength -and
         (Test-Path -LiteralPath $Dir -PathType Container) -and
         -not (Get-ChildItem -LiteralPath $Dir -Force | Select-Object -First 1)) {
    Remove-Item -LiteralPath $Dir -Force
    $Dir = Split-Path -Parent $Dir
  }
}

# The folder of the unzipped release that has tools/update.ps1: the zip's top level or one folder down
function Find-Release([string]$Dir) {
  $folders = @($Dir) + @(Get-ChildItem -LiteralPath $Dir -Directory | ForEach-Object { $_.FullName })
  foreach ($folder in $folders) {
    if (Test-Path -LiteralPath (Join-RwPath $folder 'tools/update.ps1') -PathType Leaf) { return $folder }
  }
  return $null
}

# Step 1: get the new release, then hand over to its copy of this script
function Invoke-Fetch {
  $projectDir = Split-Path -Parent $PSScriptRoot
  if (-not (Test-RwProject $projectDir)) {
    Stop-Update "$projectDir doesn't look like an RW-Template project (it needs a custom folder and a makefile)."
  }
  $temp = Join-Path ([IO.Path]::GetTempPath()) ('rw-update-' + [Guid]::NewGuid().ToString('N').Substring(0, 8))
  New-Item -ItemType Directory -Path $temp | Out-Null
  $code = 1
  try {
    $zipFile = Join-Path $temp 'RW-Template.zip'
    if ($Zip) {
      if (-not (Test-Path -LiteralPath $Zip -PathType Leaf)) { Stop-Update "can't find $Zip." }
      Copy-Item -LiteralPath $Zip -Destination $zipFile
    } else {
      Write-Host 'Downloading the newest RW-Template from GitHub...'
      # Windows PowerShell 5 only uses TLS 1.2, which GitHub needs, when asked
      [Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12
      try {
        Invoke-WebRequest -Uri "$RepoUrl/releases/latest/download/RW-Template.zip" -OutFile $zipFile -UseBasicParsing
      } catch {
        Stop-Update "couldn't download it. Check your internet connection and try again (right after a release comes out, give it a few minutes)."
      }
    }
    $newDir = Join-Path $temp 'new'
    try { Expand-Archive -LiteralPath $zipFile -DestinationPath $newDir } catch { Stop-Update "couldn't unzip the release." }
    $release = Find-Release $newDir
    if (-not $release) {
      Stop-Update "this release can't update projects (it has no tools/update.ps1). Projects from 4.0.0 and older update by hand: $DocsUrl"
    }
    & (Join-RwPath $release 'tools/update.ps1') -Apply $release -Project $projectDir
    $code = $LASTEXITCODE
  } finally {
    Remove-Item -LiteralPath $temp -Recurse -Force -ErrorAction SilentlyContinue
  }
  exit $code
}

# Step 2, run from the new release: update the project's files
function Invoke-Apply([string]$ReleaseDir, [string]$ProjectDir) {
  $ReleaseDir = (Resolve-Path -LiteralPath $ReleaseDir).Path
  $ProjectDir = (Resolve-Path -LiteralPath $ProjectDir).Path
  if (-not (Test-RwProject $ProjectDir)) {
    Stop-Update "$ProjectDir doesn't look like an RW-Template project (it needs a custom folder and a makefile)."
  }
  if (Test-Path -LiteralPath (Join-RwPath $ProjectDir 'tools/build-release.sh')) {
    Stop-Update 'this is a copy of the RW-Template repository (it has tools/build-release.sh), and updating would overwrite your work in it. Use git pull instead, or delete tools/build-release.sh to update anyway.'
  }
  $newListFile = Join-RwPath $ReleaseDir 'tools/rw-files.txt'
  $oldListFile = Join-RwPath $ProjectDir 'tools/rw-files.txt'
  if (-not (Test-Path -LiteralPath $newListFile -PathType Leaf)) { Stop-Update 'the new release has no tools/rw-files.txt.' }

  $new = Read-RwList $newListFile
  $oldVersion = ''
  $oldFiles = New-Object System.Collections.Specialized.OrderedDictionary
  if (Test-Path -LiteralPath $oldListFile -PathType Leaf) {
    $old = Read-RwList $oldListFile
    $oldVersion = $old.Version
    $oldFiles = $old.Files
  }
  if ($oldVersion -and $oldVersion -ceq $new.Version) {
    Write-Host "Already up to date: this project has RW-Template $($new.Version)."
    exit 0
  }

  # The project's copies of RW's files right now
  $current = New-Object 'System.Collections.Generic.Dictionary[string,string]'
  foreach ($path in @($new.Files.Keys) + @($oldFiles.Keys)) {
    $file = Join-RwPath $ProjectDir $path
    if (-not $current.ContainsKey($path) -and (Test-Path -LiteralPath $file -PathType Leaf)) {
      $current[$path] = Get-RwHash $file
    }
  }

  # What to do with each file. A copy that matches the old version is RW's, so it's replaced
  # (or removed) as is; anything else is saved to rw-backup first. Removals go first, so a file
  # that only changed case isn't deleted after its new version is copied in.
  $plan = New-Object System.Collections.ArrayList
  foreach ($path in $oldFiles.Keys) {
    if ($new.Files.Contains($path) -or -not $current.ContainsKey($path)) { continue }
    [void]$plan.Add([pscustomobject]@{ Action = 'remove'; Path = $path; Backup = ($current[$path] -cne $oldFiles[$path]) })
  }
  foreach ($path in $new.Files.Keys) {
    if (-not $current.ContainsKey($path)) {
      [void]$plan.Add([pscustomobject]@{ Action = 'add'; Path = $path; Backup = $false })
    } elseif ($current[$path] -cne $new.Files[$path]) {
      $unchanged = $oldFiles.Contains($path) -and $current[$path] -ceq $oldFiles[$path]
      [void]$plan.Add([pscustomobject]@{ Action = 'update'; Path = $path; Backup = -not $unchanged })
    }
  }

  if ($plan.Count -eq 0) {
    Copy-RwFile $newListFile $oldListFile
    Write-Host "Already up to date: this project has RW-Template $($new.Version)."
    exit 0
  }
  if ($oldVersion) {
    Write-Host "Updating RW-Template $oldVersion -> $($new.Version)"
  } else {
    Write-Host "Updating to RW-Template $($new.Version)"
  }

  # Settings with defaults before the update, to list the new ones at the end
  $oldSettings = New-Object 'System.Collections.Generic.HashSet[string]'
  foreach ($setting in (Get-RwDefaults $ProjectDir)) { [void]$oldSettings.Add($setting.Name) }

  $backupName = if ($oldVersion) { $oldVersion } else { 'old' }
  $backup = "rw-backup/$($backupName)-$(Get-Date -Format 'yyyyMMdd-HHmmss')"
  $updated = 0
  $added = 0
  $removed = 0
  $saved = New-Object System.Collections.ArrayList
  $failed = New-Object System.Collections.ArrayList
  foreach ($step in $plan) {
    $file = Join-RwPath $ProjectDir $step.Path
    try {
      if ($step.Backup -and (Test-Path -LiteralPath $file -PathType Leaf)) {
        Copy-RwFile $file (Join-RwPath $ProjectDir "$backup/$($step.Path)")
        [void]$saved.Add($step.Path)
      }
      if ($step.Action -eq 'remove') {
        Remove-Item -LiteralPath $file -Force
        Remove-EmptyFolders (Split-Path -Parent $file) $ProjectDir
        $removed++
      } else {
        Copy-RwFile (Join-RwPath $ReleaseDir $step.Path) $file
        if ($step.Action -eq 'add') { $added++ } else { $updated++ }
      }
    } catch {
      [void]$failed.Add($step.Path)
    }
  }

  if ($failed.Count -gt 0) {
    Write-Host "Couldn't update these files:"
    foreach ($path in $failed) { Write-Host "  $path" }
    Stop-Update 'close any program that has them open, then run the update again.'
  }
  # Saved last, so an update that stopped partway runs again from the old version
  Copy-RwFile $newListFile $oldListFile
  # Start the next build from scratch, so nothing built from the old version is left over
  # (files copied from the zip keep its dates, which can be older than the last build)
  $buildDir = Join-Path $ProjectDir 'build'
  if (Test-Path -LiteralPath $buildDir) {
    try { Remove-Item -LiteralPath $buildDir -Recurse -Force } catch { Write-Host "Couldn't delete the build folder. Delete it before you build." }
  }

  Write-Host ''
  Write-Host "Files updated: $updated, added: $added, removed: $removed. Your custom folder wasn't changed."
  if ($saved.Count -gt 0) {
    Write-Host ''
    if ($oldVersion) {
      Write-Host "You had changed these RW files. They have the new versions now, and your copies are in ${backup}:"
    } else {
      Write-Host "These files changed. Without tools/rw-files.txt the updater can't tell whether you edited them, so your copies are in ${backup}:"
    }
    foreach ($path in $saved) { Write-Host "  $path" }
  }

  # New settings: defaults that weren't there before and that custom/ doesn't mention yet
  $customFiles = @(Get-ChildItem -LiteralPath (Join-Path $ProjectDir 'custom') -Recurse -File)
  $customText = ($customFiles | ForEach-Object { [IO.File]::ReadAllText($_.FullName) }) -join "`n"
  $newSettings = New-Object System.Collections.ArrayList
  foreach ($setting in (Get-RwDefaults $ReleaseDir)) {
    if ($oldSettings.Contains($setting.Name)) { continue }
    if ($customText -cmatch ('\b' + [regex]::Escape($setting.Name) + '\b')) { continue }
    [void]$newSettings.Add($setting.Line)
  }
  if ($newSettings.Count -gt 0) {
    Write-Host ''
    Write-Host "New settings in $($new.Version). They work at their defaults; to change one, copy its line into custom/src/robot-config.cpp:"
    foreach ($line in $newSettings) { Write-Host "  $line" }
  }

  Write-Host ''
  Write-Host "What's new: $RepoUrl/releases/tag/$($new.Version)"
  Write-Host 'Done. Build and download as usual. The first build takes longer because it rebuilds everything.'
  exit 0
}

if ($Apply) { Invoke-Apply $Apply $Project } else { Invoke-Fetch }
