# Get the directory of the current script
$ShellDir = Split-Path -Parent $MyInvocation.MyCommand.Definition

# Navigate to the dockerfiles directory
Set-Location -Path "$ShellDir\dockerfiles"

# Set a prefix for the image name
$NamePrefix = "ros2-docker-"

# Retrieve the latest commit ID of the Git repository
$LatestCommitID = git -C $ShellDir rev-parse --short HEAD

# Assign the image name dynamically based on the commit ID
$NameImage = "$NamePrefix$LatestCommitID"

# Check if the Docker image already exists
if (docker image ls -q $NameImage) {
    Write-Host "Docker image is already built!"
    exit
}

# Build the Docker image
& "$ShellDir\launch_container.ps1" "build"
