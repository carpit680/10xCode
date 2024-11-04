# _Containered ROS2 Development Environment_

Credits to [tatsuyai713](https://github.com/tatsuyai713/Development-Container-for-ROS2-on-Arm64-Mac) It gave me a good starting point.


These steps will guide you through setting up and using a ROS 2 Docker container on your x86-64 or arm64-based Ubuntu or Mac device, including desktop access through RDP or passthrough if using vscode dev-containers.

## Method1: Dev Container in vscode with GUI passthrough (Tested on x86-64 Ubuntu 22.04)
This sets up ROS2 Dev Container with Rviz2, Ignition Gazebo, Moveit2, Nav2, SLAM Toolbox and other dependencies.

### Setup Instructions

1. Install Dev Container VSCode extension and Remote Development Extension pack

2. Clone the repository.

3. VSCode should ask you to reopen the directory in dev container. Or you can invoke `Dev Containers: Reopen in Container` from command panel in VSCode.

4. You will need to source ROS2 using by running `source /opt/ros/humble/setup.bash` for now.

5. Any GUI application can then directly be run using the terminal.
 
## Method2: Development Container for ROS 2 via RDP (Tested on x86-64 Ubuntu 22.04 and arm64 M2 Mac)
This Dockerfile is designed to create a Docker container for ROS 2 Humble and includes the following main packages preinstalled.

- ROS2 Humble
- KDE Plasma
- Gazebo Ignition
- Moveit2
- SLAM Toolbox
- Nav2

### Prerequisites
- Docker installed on your system
- User added to docker group

### Setup Instructions

#### 1. **Install Docker**:
Follow the official Docker installation guide.

#### 2. **Build the Container**:
Run the build script to build the Docker image and install all dependencies. This should take around 10-20 minutes. 


```bash
   cd docker
   ./build_container.sh
```

#### 3. **Start the Container**:
Initiate the container with

```bash
   cd docker
   ./start_container.sh
```

#### 4. **Commit Changes**:
Save the current state of the Docker container using

```bash
   cd docker
   ./stop_container.sh
```

#### 5. **Access the Container**:
For terminal access, run

```bash
   cd docker
   ./attach_container.sh
```

#### 6. **Desktop Environment**: 
   To use the KDE Plasma Desktop via xrdp, connect using an RDP client to `127.0.0.1` or `localhost`.

### Recommended RDP Clients

- **Windows App / Microsoft Remote Desktop**:
   Available on Mac and Windows. Use the host system's username and password.

- **Remmina**:
   Available on Ubuntu. Use the host system's username and password.
