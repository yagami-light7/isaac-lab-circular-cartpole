param(
    [string]$EnvPath = ""
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent (Split-Path -Parent $scriptDir)
$envFile = Join-Path $repoRoot "environment.host_runtime.yml"

if ([string]::IsNullOrWhiteSpace($EnvPath)) {
    $EnvPath = Join-Path $repoRoot ".conda\\cartpole-s2r-host"
}

$env:CONDA_NO_PLUGINS = "true"

if (-not (Test-Path $envFile)) {
    throw "Cannot find environment file: $envFile"
}

if (Test-Path $EnvPath) {
    Write-Host "Updating host runtime environment at $EnvPath"
    conda env update -p $EnvPath -f $envFile --prune
} else {
    Write-Host "Creating host runtime environment at $EnvPath"
    conda env create -p $EnvPath -f $envFile
}

Write-Host ""
Write-Host "Environment ready."
Write-Host "Activate with:"
Write-Host "  conda activate $EnvPath"
