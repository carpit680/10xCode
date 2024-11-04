FROM ubuntu:22.04
ARG UID=9001
ARG GID=9001
ARG UNAME=ubuntu
ARG HOSTNAME=docker
ARG NEW_HOSTNAME=${HOSTNAME}-Docker
ARG USERNAME=$UNAME
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
        supervisor \
        htop

USER root

# ENV XDG_CURRENT_DESKTOP KDE
ENV KWIN_COMPOSE N
# Use sudoedit to change protected files instead of using sudo on kate
ENV SUDO_EDITOR kate

# install ROS2 Humble
RUN sudo apt update && sudo apt install locales
RUN sudo locale-gen en_US en_US.UTF-8
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
RUN sudo apt install software-properties-common
RUN sudo add-apt-repository universe
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN sudo apt update && sudo apt upgrade -y 
RUN sudo apt install ros-humble-desktop -y
RUN sudo apt install ros-dev-tools -y 

# Install ROS tools
RUN sudo apt update && sudo apt install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        && \
    sudo apt clean && \
    sudo rm -rf /var/lib/apt/lists/*

# initialize rosdep
RUN sudo rosdep init && \
    rosdep update

RUN wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | zsh || true

RUN echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh" >> ~/.zshrc

RUN echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.zshrc
RUN echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc

RUN sudo apt update && sudo apt install ros-humble-moveit -y

RUN sudo apt install ros-humble-rmw-cyclonedds-cpp -y

RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.zshrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrcc

RUN sudo apt install -y ros-humble-navigation2 \
    ros-humble-nav2-bringup

RUN sudo apt install ros-humble-slam-toolbox -y

RUN sudo apt update && sudo apt install ros-humble-ros-gz -y 

RUN sudo apt-get update \
  && sudo apt-get upgrade -y \
  && sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
    dirmngr \
    cmake \
  && sudo apt-get clean

RUN mkdir -p /home/$USERNAME/ros2_ws/src \
    && cd /home/$USERNAME/ros2_ws/src \
    && git clone https://github.com/ros-controls/gz_ros2_control/ -b humble \
    && rosdep install --from-paths ./ -i -y --rosdistro humble

RUN cd /home/$USERNAME/ros2_ws/ \
  && . /opt/ros/humble/setup.sh \
  && colcon build --merge-install

RUN echo "source /home/$USERNAME/ros2_ws/install/setup.zsh" >> /home/$USERNAME/.zshrc
RUN echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "eval '$(register-python-argcomplete3 ros2)'" >> /home/$USERNAME/.zshrc
RUN echo "eval '$(register-python-argcomplete3 colcon)'" >> /home/$USERNAME/.zshrc  

USER root

RUN { \
      echo '#DPkg::Post-Invoke { "rm -f /var/cache/apt/archives/*.deb /var/cache/apt/archives/partial/*.deb /var/cache/apt/*.bin || true"; };'; \
      echo '#APT::Update::Post-Invoke { "rm -f /var/cache/apt/archives/*.deb /var/cache/apt/archives/partial/*.deb /var/cache/apt/*.bin || true"; };'; \
      echo '#Dir::Cache::pkgcache ""; Dir::Cache::srcpkgcache "";'; \
    } > /etc/apt/apt.conf.d/docker-clean

USER $USERNAME
RUN LANG=C xdg-user-dirs-update --force

USER root
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/*
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.config

# Copy entrypoint script
COPY dev-container-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/dev-container-entrypoint.sh
ENTRYPOINT ["dev-container-entrypoint.sh"]
