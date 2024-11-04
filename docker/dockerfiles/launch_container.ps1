# Get the directory of the current script
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition

# Set a prefix for the image name
$NamePrefix = "ros2-docker-"

# Retrieve the latest commit ID of the Git repository
$LatestCommitID = git -C $ScriptDir rev-parse --short HEAD

# Assign the image name dynamically based on the commit ID
$NameImage = "$NamePrefix$LatestCommitID"

# Check if the Docker image exists
$ImageExists = docker image ls -q $NameImage

# Arguments handling
$ArgsCount = $Args.Count
$FirstArg = $Args[0]

if (-not $ImageExists) {
    if ($ArgsCount -eq 1 -and $FirstArg -eq "build") {
        Write-Host "Image $NameImage does not exist."
        Write-Host "Now building image without proxy..."
        docker build --file=./noproxy.dockerfile -t $NameImage . --build-arg UID=$(id -u) --build-arg GID=$(id -u) --build-arg UNAME=$Env:USERNAME --build-arg LOCALE="US"
        exit
    } else {
        Write-Host "Docker image not found. Please set up first!"
        exit
    }
} elseif ($ArgsCount -eq 1 -and $FirstArg -eq "build") {
    Write-Host "Docker image found. Please select mode!"
    exit
}

# Additional Commands (Commit, Stop, Delete)
if ($ArgsCount -eq 1 -and $FirstArg -eq "commit") {
    docker commit $NameImage "$NameImage:latest"
    $ContainerID = docker ps -a -f "name=$NameImage-docker" --format "{{.ID}}"
    docker rm $ContainerID -f
    exit
} elseif ($ArgsCount -eq 1 -and $FirstArg -eq "stop") {
    $ContainerID = docker ps -a -f "name=$NameImage-docker" --format "{{.ID}}"
    docker stop $ContainerID
    docker rm $ContainerID -f
    exit
} elseif ($ArgsCount -eq 1 -and $FirstArg -eq "delete") {
    Write-Host "Now deleting docker container..."
    $ContainerID = docker ps -a -f "name=$NameImage-docker" --format "{{.ID}}"
    docker stop $ContainerID
    docker rm $ContainerID -f
    docker image rm $NameImage
    exit
}

# Docker options for XWindow and Display
$DockerOpt = "--env=QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY=$Env:COMPUTERNAME:0 --shm-size=4gb --env=TERM=xterm-256color -w /home/$Env:USERNAME -u $Env:USERNAME -p 3389:3389 -e PASSWD=$Env:USERNAME"
$DockerName = "$NameImage-docker"

# Running Docker container
$ContainerID = docker ps -a -f "name=$DockerName" --format "{{.ID}}"
if (-not $ContainerID) {
    if ($ArgsCount -eq 1 -and $FirstArg -eq "xrdp") {
        Write-Host "Remote Desktop Mode"
        docker run $DockerOpt --name=$DockerName --entrypoint docker-entrypoint.sh "$NameImage:latest"
    } else {
        docker run $DockerOpt --name=$DockerName -it --entrypoint /bin/bash "$NameImage:latest"
    }
} else {
    docker start $ContainerID
    docker exec -it $ContainerID /bin/bash
}
