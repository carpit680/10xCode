# Get the directory of the current script
$ShellDir = Split-Path -Parent $MyInvocation.MyCommand.Definition

# Navigate to the dockerfiles directory
Set-Location -Path "$ShellDir\dockerfiles"

# Execute launch_container.ps1 with the "commit" argument
& "$ShellDir\launch_container.ps1" "commit"
