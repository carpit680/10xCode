# __Development Container for ROS 2 via RDP (Tested on amd64 Ubuntu and arm64 M2 Mac)__
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

1. **Start the Container**:
Initiate the container with

```bash
   cd docker
   ./start_container.sh
```

4. **Commit Changes**:
Save the current state of the Docker container using

```bash
   cd docker
   ./stop_container.sh
```

5. **Access the Container**:
For terminal access, run

```bash
   cd docker
   ./attach_container.sh
```

   to use bash inside the Docker container.

6. **Desktop Environment**: 
   To use the KDE Plasma Desktop via xrdp, connect using an RDP client to `127.0.0.1` or `localhost`.

### Recommended RDP Clients

- **Windows App**:
  It supports sound playback. Use the host system's username and password.

These steps will guide you through setting up and using a ROS 2 Docker container on your amd64 or arm64-based Ubuntu or Mac device, including desktop access through RDP.
