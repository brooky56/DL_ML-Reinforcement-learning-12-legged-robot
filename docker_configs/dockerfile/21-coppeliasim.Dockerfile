USER root
EXPOSE 19997

ARG COPPELIASIM_RELEASE=CoppeliaSim_Edu_V4_1_0_Ubuntu18_04

# installing last version of cmake
SHELL ["/bin/bash", "-c"] 
RUN \

    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null &&\
    apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main' && \
    # apt update && apt-get install -q -y \ 
    # cmake \
    # xsltproc \
    #  && \
  rm -rf /var/lib/apt/lists/*


RUN \
        echo "Downloading and unpacking $COPPELIASIM_RELEASE" && \
        cd /opt && \
        curl --progress-bar --remote-name --location \
            https://www.coppeliarobotics.com/files/$COPPELIASIM_RELEASE.tar.xz || exit 1 && \
            tar -xf /opt/$COPPELIASIM_RELEASE.tar.xz -C /opt && \
        rm /opt/$COPPELIASIM_RELEASE.tar.xz 
SHELL ["/bin/sh", "-c"]

# COPY docker_configs/CoppeliaSim_so_files/* /opt/${COPPELIASIM_RELEASE}/

# Installing the new version of libPlugin, which has needed packages for ros
# SHELL ["/bin/bash", "-c"]
# RUN \
#     cd /opt/${COPPELIASIM_RELEASE}/programming && \
#     rm -rf libPlugin &&\
#     git clone https://github.com/CoppeliaRobotics/libPlugin.git &&\
#     cd libPlugin &&\
#     git checkout melodic
# SHELL ["/bin/sh", "-c"]

USER ${APP_USER}
SHELL ["/bin/bash", "-c"]
RUN \
    echo "export PATH=\$PATH:/opt/$COPPELIASIM_RELEASE" >> ~/.bashrc &&\
    # echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/opt/$COPPELIASIM_RELEASE" >> ~/.bashrc &&\
    echo "export COPPELIASIM_ROOT_DIR=/opt/$COPPELIASIM_RELEASE" >> ~/.bashrc
SHELL ["/bin/sh", "-c"]

