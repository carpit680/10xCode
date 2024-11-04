FROM ubuntu:22.04
ARG UID=9001
ARG GID=9001
ARG UNAME=ubuntu
ARG HOSTNAME=docker
ARG NEW_HOSTNAME=${HOSTNAME}-Docker
ARG USERNAME="$UNAME"
ARG HOME=/home/$USERNAME
ARG LOCALE="US"


RUN useradd -u $UID -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        mkdir /etc/sudoers.d && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        usermod  --uid $UID $USERNAME && \
        groupmod --gid $GID $USERNAME && \
        chown -R $USERNAME:$USERNAME $HOME && \
        chmod 666 /dev/null && \
        chmod 666 /dev/urandom


RUN echo 'path-include=/usr/share/locale/ja/LC_MESSAGES/*.mo' > /etc/dpkg/dpkg.cfg.d/includes \
    && apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
        sudo \
        build-essential \
        curl \
        less \
        apt-utils \
        tzdata \
        git \
        tmux \
        screen \
        bash-completion \
        command-not-found \
        libglib2.0-0 \
        vim \
        emacs \
        ssh \
        rsync \
        python3-pip \
        sed \
        ca-certificates \
        wget \
        lsb-release \
        gnupg

RUN apt update && DEBIAN_FRONTEND=noninteractive apt install --no-install-recommends -y \
        software-properties-common \
        alsa-base \
        alsa-utils \
        apt-transport-https \
        apt-utils \
        build-essential \
        ca-certificates \
        cups-filters \
        cups-common \
        cups-pdf \
        curl \
        file \
        wget \
        bzip2 \
        gzip \
        p7zip-full \
        xz-utils \
        zip \
        unzip \
        zstd \
        gcc \
        git \
        jq \
        make \
        python3 \
        python3-cups \
        python3-numpy \
        mlocate \
        nano \
        vim \
        htop \
        fonts-dejavu-core \
        fonts-freefont-ttf \
        fonts-noto \
        fonts-noto-cjk \
        fonts-noto-cjk-extra \
        fonts-noto-color-emoji \
        fonts-noto-hinted \
        fonts-noto-mono \
        fonts-opensymbol \
        fonts-symbola \
        fonts-ubuntu \
        libpulse0 \
        pulseaudio \
        supervisor \
        net-tools \
        libglvnd-dev \
        libgl1-mesa-dev \
        libegl1-mesa-dev \
        libgles2-mesa-dev \
        libglvnd0 \
        libgl1 \
        libglx0 \
        libegl1 \
        libgles2 \
        libglu1 \
        libsm6 \
        vainfo \
        vdpauinfo \
        pkg-config \
        mesa-utils \
        mesa-utils-extra \
        va-driver-all \
        xserver-xorg-input-all \
        xserver-xorg-video-all \
        mesa-vulkan-drivers \
        libvulkan-dev \
        libxau6 \
        libxdmcp6 \
        libxcb1 \
        libxext6 \
        libx11-6 \
        libxv1 \
        libxtst6 \
        xdg-utils \
        dbus-x11 \
        libdbus-c++-1-0v5 \
        xkb-data \
        x11-xkb-utils \
        x11-xserver-utils \
        x11-utils \
        x11-apps \
        xauth \
        xbitmaps \
        xinit \
        xfonts-base \
        libxrandr-dev \
        vulkan-tools \
        zsh && \
    rm -rf /var/lib/apt/lists/*

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
        # xrdp \
        supervisor \
        htop

USER root

# ENV XDG_CURRENT_DESKTOP KDE
ENV KWIN_COMPOSE N
# Use sudoedit to change protected files instead of using sudo on kate
ENV SUDO_EDITOR kate

# install ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt install -y --no-install-recommends \
        ros-humble-desktop \
        ros-dev-tools \
        && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# install colcon and rosdep
RUN apt update && apt install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# Pulseaudio
RUN apt update && apt install -y libtool autoconf pkg-config libssl-dev libpam0g-dev libx11-dev libxfixes-dev libxrandr-dev nasm xsltproc flex bison libxml2-dev dpkg-dev libcap-dev meson ninja-build libsndfile1-dev libtdb-dev check doxygen libxml-parser-perl

USER $USERNAME

# initialize rosdep
RUN sudo rosdep init && \
    rosdep update

RUN wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | zsh || true

RUN echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh" >> ~/.zshrc

RUN echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.zshrc

# Install ROS tools
RUN sudo apt install ros-humble-moveit -y

RUN sudo apt install ros-humble-rmw-cyclonedds-cpp -y

RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.zshrc

RUN sudo apt install -y ros-humble-navigation2 \
    ros-humble-nav2-bringup

RUN sudo apt install ros-humble-slam-toolbox -y

RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update
RUN sudo apt-get install -y gz-harmonic

USER root

RUN { \
      echo '#DPkg::Post-Invoke { "rm -f /var/cache/apt/archives/*.deb /var/cache/apt/archives/partial/*.deb /var/cache/apt/*.bin || true"; };'; \
      echo '#APT::Update::Post-Invoke { "rm -f /var/cache/apt/archives/*.deb /var/cache/apt/archives/partial/*.deb /var/cache/apt/*.bin || true"; };'; \
      echo '#Dir::Cache::pkgcache ""; Dir::Cache::srcpkgcache "";'; \
    } > /etc/apt/apt.conf.d/docker-clean

USER ${USER}
RUN LANG=C xdg-user-dirs-update --force

USER root
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/*
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.config

# Copy entrypoint script
COPY dev-container-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/dev-container-entrypoint.sh
ENTRYPOINT ["dev-container-entrypoint.sh"]
