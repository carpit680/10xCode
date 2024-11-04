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

RUN usermod --shell /usr/bin/zsh $USERNAME

RUN apt update && DEBIAN_FRONTEND=noninteractive apt install --no-install-recommends -y \
        kde-plasma-desktop \
        kwin-addons \
        kwin-x11 \
        kdeadmin \
        akregator \
        ark \
        baloo-kf5 \
        breeze-cursor-theme \
        breeze-icon-theme \
        debconf-kde-helper \
        colord-kde \
        desktop-file-utils \
        filelight \
        gwenview \
        hspell \
        kaddressbook \
        kaffeine \
        kate \
        kcalc \
        kcharselect \
        kdeconnect \
        kde-spectacle \
        kde-config-screenlocker \
        kde-config-updates \
        kdf \
        kget \
        kgpg \
        khelpcenter \
        khotkeys \
        kimageformat-plugins \
        kinfocenter \
        kio-extras \
        kleopatra \
        kmail \
        kmenuedit \
        kmix \
        knotes \
        kontact \
        kopete \
        korganizer \
        krdc \
        ktimer \
        kwalletmanager \
        librsvg2-common \
        okular \
        okular-extra-backends \
        plasma-dataengines-addons \
        plasma-discover \
        plasma-runners-addons \
        plasma-wallpapers-addons \
        plasma-widgets-addons \
        plasma-workspace-wallpapers \
        qtvirtualkeyboard-plugin \
        sonnet-plugins \
        sweeper \
        systemsettings \
        xdg-desktop-portal-kde \
        kubuntu-restricted-extras \
        kubuntu-wallpapers \
        pavucontrol-qt \
        transmission-qt && \
    rm -rf /var/lib/apt/lists/*

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
        xrdp \
        supervisor \
        htop

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
      ubuntu-wallpapers \
    && apt clean \
    && rm -rf /var/cache/apt/archives/* \
    && rm -rf /var/lib/apt/lists/* 

USER root

ENV XDG_CURRENT_DESKTOP=KDE
ENV KWIN_COMPOSE=N
# Use sudoedit to change protected files instead of using sudo on kate
ENV SUDO_EDITOR=kate

RUN add-apt-repository ppa:mozillateam/ppa

RUN { \
      echo 'Package: firefox*'; \
      echo 'Pin: release o=LP-PPA-mozillateam'; \
      echo 'Pin-Priority: 1001'; \
      echo ' '; \
      echo 'Package: firefox*'; \
      echo 'Pin: release o=Ubuntu*'; \
      echo 'Pin-Priority: -1'; \
    } > /etc/apt/preferences.d/99mozilla-firefox

RUN apt -y update \
 && apt install -y firefox

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
RUN apt update && apt install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# initialize rosdep
RUN sudo rosdep init && \
    rosdep update

RUN wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | zsh || true

RUN echo "source /opt/ros/humble/setup.zsh" >> /home/$USERNAME/.zshrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh" >> /home/$USERNAME/.zshrc

RUN echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> /home/$USERNAME/.zshrc

RUN sudo apt update && sudo apt install ros-humble-moveit -y

RUN sudo apt install ros-humble-rmw-cyclonedds-cpp -y

RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.zshrc

RUN sudo apt install -y ros-humble-navigation2 \
    ros-humble-nav2-bringup

RUN sudo apt install ros-humble-slam-toolbox -y

RUN sudo apt update && sudo apt install ros-humble-ros-gz -y 

RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    cmake \
  && apt-get clean

RUN mkdir -p /home/$USERNAME/ros2_ws/src \
    && cd /home/$USERNAME/ros2_ws/src \
    && git clone https://github.com/ros-controls/gz_ros2_control/ -b humble \
    && rosdep install --from-paths ./ -i -y --rosdistro humble

RUN cd /home/$USERNAME/ros2_ws/ \
  && . /opt/ros/humble/setup.sh \
  && colcon build --merge-install

RUN echo "source /home/$USERNAME/ros2_ws/install/setup.zsh" >> /home/$USERNAME/.zshrc

USER root

# Expose RDP port
EXPOSE 3389

RUN echo "startplasma-x11" > /etc/skel/.xsession \
    && install -o root -g xrdp -m 2775 -d /var/run/xrdp \
    && install -o root -g xrdp -m 3777 -d /var/run/xrdp/sockdir \
    && install -o root -g root -m 0755 -d /var/run/dbus

# Set supervisord conf for xrdp service
RUN { \
      echo "[supervisord]"; \
      echo "user=root"; \
      echo "nodaemon=true"; \
      echo "logfile=/var/log/supervisor/supervisord.log"; \
      echo "childlogdir=/var/log/supervisor"; \
      echo "[program:dbus]"; \
      echo "command=/usr/bin/dbus-daemon --system --nofork --nopidfile"; \
      echo "[program:xrdp-sesman]"; \
      echo "command=/usr/sbin/xrdp-sesman --nodaemon"; \
      echo "[program:xrdp]"; \
      echo "command=/usr/sbin/xrdp --nodaemon"; \
      echo "user=xrdp"; \
      echo "[program:pulseaudio]"; \
      echo "priority=15"; \
      echo "directory=/home/$USERNAME"; \
      echo "command=/usr/bin/pulseaudio"; \
      echo "user=$USERNAME"; \
      echo "autostart=true"; \
      echo "autorestart=true"; \
      echo "stopsignal=TERM"; \
      echo "environment=DISPLAY=:1,HOME=/home/$USERNAME"; \
      echo "stdout_logfile=/var/log/pulseaudio.log"; \
      echo "stderr_logfile=/var/log/pulseaudio.err"; \
    } > /etc/supervisor/xrdp.conf

RUN { \
      echo '#DPkg::Post-Invoke { "rm -f /var/cache/apt/archives/*.deb /var/cache/apt/archives/partial/*.deb /var/cache/apt/*.bin || true"; };'; \
      echo '#APT::Update::Post-Invoke { "rm -f /var/cache/apt/archives/*.deb /var/cache/apt/archives/partial/*.deb /var/cache/apt/*.bin || true"; };'; \
      echo '#Dir::Cache::pkgcache ""; Dir::Cache::srcpkgcache "";'; \
    } > /etc/apt/apt.conf.d/docker-clean

USER $USERNAME
RUN LANG=C xdg-user-dirs-update --force
RUN touch /home/${USERNAME}/Desktop/home.desktop
RUN touch /home/${USERNAME}/Desktop/trash.desktop

# Make Desktop Icons
RUN { \
    echo "[Desktop Entry]"; \
    echo "Encoding=UTF-8"; \
    echo "Name=Home"; \
    echo "GenericName=Personal Files"; \
    echo "URL[$e]=$HOME"; \
    echo "Icon=user-home"; \
    echo "Type=Link"; \
    } > /home/${USERNAME}/Desktop/home.desktop

RUN { \
    echo "[Desktop Entry]"; \
    echo "Name=Trash"; \
    echo "Comment=Contains removed files"; \
    echo "Icon=user-trash-full"; \
    echo "EmptyIcon=user-trash"; \
    echo "URL=trash:/"; \
    echo "Type=Link"; \
    } > /home/${USERNAME}/Desktop/trash.desktop

USER root
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/*
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.config

# Copy entrypoint script
COPY docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh
ENTRYPOINT ["docker-entrypoint.sh"]
