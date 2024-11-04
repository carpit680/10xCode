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

# Check if any Docker containers match the name prefix
$ContainerExists = docker ps -a | Select-String -Pattern $NamePrefix

if ($ContainerExists) {
    Write-Host "Docker container restarting..."
    $ContainerID = docker ps -a -f "name=$NamePrefix" --format "{{.ID}}"
    
    # Set up X11 authentication
    $XauthPath = "C:\tmp\.docker.xauth"
    if (Test-Path $XauthPath) {
        Remove-Item $XauthPath -Force
    }
    New-Item -Path $XauthPath -ItemType File | Out-Null
    $XauthList = & xauth nlist :0 | ForEach-Object { $_ -replace '^....', 'ffff' }
    if ($XauthList) {
        $XauthList | xauth -f $XauthPath nmerge -
    }
    Set-Acl -Path $XauthPath -AclObject (New-Object System.Security.AccessControl.FileSecurity).SetAccessRule((New-Object System.Security.AccessControl.FileSystemAccessRule("Everyone", "Read", "Allow")))

    # Start the Docker container
    docker start $ContainerID
    exit
}

# Run launch_container.ps1 in xrdp mode with nohup equivalent
Start-Process -FilePath "powershell" -ArgumentList "./launch_container.ps1 xrdp" -NoNewWindow -RedirectStandardOutput "C:\tmp\nohup.out" -RedirectStandardError "C:\tmp\nohup.err"
