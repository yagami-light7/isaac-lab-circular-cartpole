$ErrorActionPreference = "Stop"

# Create or update the local conda environment used by the Windows host runtime.
# Keep the environment inside the project directory so it is easy to move with the repo.

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = Split-Path -Parent $scriptRoot
$envFile = Join-Path $projectRoot "environment-host-runtime.yml"
$envPrefix = Join-Path $projectRoot ".conda\\cartpole-s2r-host"

if (-not (Test-Path $envFile)) {
    throw "Environment file not found: $envFile"
}

# The base conda installation on this machine has a plugin compatibility warning.
# Disable plugins explicitly so env create/update works reliably.
$env:CONDA_NO_PLUGINS = "true"

if (Test-Path $envPrefix) {
    Write-Host "Existing environment detected. Updating: $envPrefix"
    conda env update --prefix $envPrefix --file $envFile --prune
}
else {
    Write-Host "Creating environment: $envPrefix"
    conda env create --prefix $envPrefix --file $envFile
}

Write-Host ""
Write-Host "Environment is ready."
Write-Host "Activate with:"
Write-Host "conda activate $envPrefix"
