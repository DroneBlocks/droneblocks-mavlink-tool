# One-shot Windows setup for the DEXI-3 flight-controller flasher.
#
# Fastest way to run it on a fresh PC (paste into PowerShell):
#   irm https://raw.githubusercontent.com/DroneBlocks/droneblocks-mavlink-tool/main/windows/setup-windows.ps1 | iex
#
# It clones/updates the repo, builds the Python venv, installs dfu-util (no
# package manager needed), downloads Zadig, and copies the launchers + Zadig
# readme to the Desktop. Idempotent - safe to re-run.
#
# After it finishes: do the ONE-TIME Zadig step (Desktop README), then
# double-click Flash-DEXI-debug.cmd.

$ErrorActionPreference = 'Stop'
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12

$RepoUrl = 'https://github.com/DroneBlocks/droneblocks-mavlink-tool.git'
$Repo    = Join-Path $env:USERPROFILE 'droneblocks-mavlink-tool'
$Bin     = Join-Path $env:USERPROFILE 'bin'
$Desktop = [Environment]::GetFolderPath('Desktop')

function Step($m) { Write-Host "==> $m" -ForegroundColor Cyan }

# ---- prerequisites -----------------------------------------------------------
foreach ($cmd in 'git', 'py', 'tar') {
    if (-not (Get-Command $cmd -ErrorAction SilentlyContinue)) {
        throw "$cmd not found. Install it first (git: git-scm.com; Python 'py' launcher: python.org / 'winget install Python.Python.3.12'; tar ships with Windows 10 1803+)."
    }
}

# ---- 1. repo -----------------------------------------------------------------
if (Test-Path (Join-Path $Repo '.git')) {
    Step "Updating repo ($Repo)"
    git -C $Repo pull --ff-only
} else {
    Step "Cloning repo to $Repo"
    git clone $RepoUrl $Repo
}

# ---- 2. venv + Python deps ---------------------------------------------------
$py = Join-Path $Repo 'venv\Scripts\python.exe'
if (-not (Test-Path $py)) {
    Step "Creating venv"
    py -m venv (Join-Path $Repo 'venv')
}
Step "Installing Python deps"
& $py -m pip install --upgrade pip -q
& $py -m pip install -r (Join-Path $Repo 'requirements.txt')

# ---- 3. dfu-util -> %USERPROFILE%\bin (no winget/choco needed) ---------------
if (-not (Test-Path (Join-Path $Bin 'dfu-util.exe'))) {
    Step "Installing dfu-util to $Bin"
    New-Item -ItemType Directory -Force -Path $Bin | Out-Null
    $tmp = Join-Path $env:TEMP 'dfu-util-dl'
    $xz  = Join-Path $env:TEMP 'dfu-util.tar.xz'
    New-Item -ItemType Directory -Force -Path $tmp | Out-Null
    Invoke-WebRequest 'https://dfu-util.sourceforge.net/releases/dfu-util-0.11-binaries.tar.xz' -OutFile $xz -UseBasicParsing
    tar -xf $xz -C $tmp
    $win64 = Get-ChildItem $tmp -Recurse -Directory | Where-Object { $_.Name -eq 'win64' } | Select-Object -First 1
    if (-not $win64) { throw 'win64 folder not found in dfu-util archive' }
    Copy-Item (Join-Path $win64.FullName '*') $Bin -Force
} else {
    Step "dfu-util already installed ($Bin)"
}
# persist bin on the user PATH (launchers also prepend it at runtime, so this
# works even before the next sign-in)
$userPath = [Environment]::GetEnvironmentVariable('Path', 'User'); if (-not $userPath) { $userPath = '' }
if ($userPath -notlike "*$Bin*") {
    [Environment]::SetEnvironmentVariable('Path', ($userPath.TrimEnd(';') + ';' + $Bin), 'User')
}

# ---- 4. Desktop: launchers + Zadig kit ---------------------------------------
Step "Copying launchers + Zadig kit to Desktop"
foreach ($f in 'Flash-DEXI.cmd', 'Flash-DEXI-debug.cmd', 'READ-ME-FIRST-Zadig-Setup.txt') {
    Copy-Item (Join-Path $Repo "windows\$f") $Desktop -Force
}
$zadig = Join-Path $Desktop 'zadig.exe'
if (-not (Test-Path $zadig)) {
    Step "Downloading Zadig"
    Invoke-WebRequest 'https://github.com/pbatard/libwdi/releases/download/v1.5.1/zadig-2.9.exe' -OutFile $zadig -UseBasicParsing
}

Write-Host ""
Write-Host "DONE - flasher is installed." -ForegroundColor Green
Write-Host "Next steps (on the Desktop):"
Write-Host "  1) ONE-TIME: follow 'READ-ME-FIRST-Zadig-Setup.txt' (installs the DFU driver)."
Write-Host "  2) Flash a board: double-click 'Flash-DEXI-debug.cmd'."
