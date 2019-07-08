#!/bin/bash

################################################################################

# Pass 'sudo' privileges if previously granted in parent scripts.
if [ ! -z "$SUDO_USER" ]; then
  export USER=$SUDO_USER
fi

################################################################################

# Install Docker Community Edition.
# https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/

# Remove older versions of Docker if any.
apt-get remove \
  docker \
  docker-engine \
  docker.io

# Gather required packages for Docker installation.
apt-get update && apt-get install -y \
  apt-transport-https \
  ca-certificates \
  curl \
  software-properties-common

# Add the official Docker GPG key.
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -
apt-key fingerprint 0EBFCD88

# Add the Docker repository.
add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu xenial stable"

# Install the latest version of Docker.
# Any existing installation will be replaced.
apt-get update && apt-get install -y \
  docker-ce

# Test the Docker installation after making sure that the service is running.
service docker stop
service docker start
while ! pgrep dockerd > /dev/null; do
  sleep 1
done
docker version
docker run --rm hello-world

################################################################################

# Add the current user to the 'docker' group to run Docker without 'sudo'.
# Logging out and back in is required for the group change to take effect.
usermod -a -G docker ${USER}

################################################################################

# Install Docker Compose.
# https://docs.docker.com/compose/install/#install-compose
# https://github.com/docker/compose/releases

# Install Docker Compose.
curl -L https://github.com/docker/compose/releases/download/1.21.0/docker-compose-`uname -s`-`uname -m` -o /usr/local/bin/docker-compose
chmod +x /usr/local/bin/docker-compose

# Test the Docker Compose installation.
docker-compose version

################################################################################

# Install Nvidia Docker 2.
# https://github.com/NVIDIA/nvidia-docker
# https://github.com/NVIDIA/nvidia-docker/wiki/Usage
# https://github.com/nvidia/nvidia-container-runtime#environment-variables-oci-spec

# Remove 'nvidia-docker' and all existing GPU containers.
docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f
apt-get purge -y nvidia-docker

# Add the Nvidia Docker package repositories.
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | tee /etc/apt/sources.list.d/nvidia-docker.list

# Install 'nvidia-docker2' and reload the Docker daemon configuration.
apt-get update && apt-get install -y \
  nvidia-docker2

# Test the Nvidia Docker installation after making sure that the service is running and that Nvidia drivers are found.
service docker stop
service docker start
while ! pgrep dockerd > /dev/null; do
  sleep 1
done
# TODO: Find why it works locally but not in GitLab CI.
if [ -e /proc/driver/nvidia/version ]; then
  docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi
fi
