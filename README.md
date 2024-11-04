# __Development Container for ROS 2 via RDP (Tested on x86 Ubuntu and arm64 M2 Mac)__
This Dockerfile is designed to create a Docker container specifically for ROS 2 Humble.

### Prerequisites
- Docker installed on your system
- User added to docker group

### Setup Instructions

#### 1. **Install Docker**:
Follow the official Docker installation guide.

#### 2. **Build the Container**:
Run the build script to build Docker image and install all dependencies. This should take around 10-20 minutes. 

```bash
   ./build_container.sh
```

1. **Start the Container**:
Initiate the container with

```bash
   ./start_container.sh
```

4. **Commit Changes**:
Save the current state of the Docker container using

```bash
   ./stop_container.sh
```

5. **Access the Container**:
For terminal access, run

```bash
   ./attach_container.sh
```

   to use bash inside the Docker container.

6. **Desktop Environment**: 
   To use the KDE Plasma Desktop via xrdp, connect using an RDP client to `127.0.0.1` or `localhost`.

### Recommended RDP Clients

- **Windows App**:
  It supports sound playback. Use host system's username as your user name and password.

These steps will guide you through setting up and using a ROS 2 Docker container on your x86, arm64 based Ubuntu, Mac or Windows device, including desktop access through RDP. Choose the Microsoft Remote Desktop for comprehensive functionality.
