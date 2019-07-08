#!/bin/bash

################################################################################

# Install Docker Community Edition.
# https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/

# Remove older versions of Docker if any.
sudo apt-get remove \
  docker \
  docker-engine \
  docker.io

# Gather required packages for Docker installation.
sudo apt-get update
sudo apt-get install -y \
  apt-transport-https \
  ca-certificates \
  curl \
  software-properties-common

# Add the official Docker GPG key.
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo apt-key fingerprint 0EBFCD88

# Add the Docker repository.
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu xenial stable"

# Install the latest version of Docker.
# Any existing installation will be replaced.
sudo apt-get update
sudo apt-get install -y \
  docker-ce

# Test the Docker installation after making sure that the service is running.
sudo service docker stop
sudo service docker start
while ! pgrep dockerd > /dev/null; do
  sleep 1
done
sudo docker version
sudo docker run --rm hello-world

################################################################################

# Add the current user to the 'docker' group to run Docker without 'sudo'.
# Logging out and back in is required for the group change to take effect.
sudo usermod -a -G docker ${USER}

################################################################################

# Install GitLab Runner.
# https://docs.gitlab.com/runner/
# https://docs.gitlab.com/runner/install/linux-repository.html

# Add GitLab official repository.
curl -L https://packages.gitlab.com/install/repositories/runner/gitlab-runner/script.deb.sh | sudo bash

# TODO: Check why permission is denied.
# Pin GitLab repository for higher priority.
cat > /etc/apt/preferences.d/pin-gitlab-runner.pref <<EOF
Explanation: Prefer GitLab provided packages over the Debian native ones
Package: gitlab-runner
Pin: origin packages.gitlab.com
Pin-Priority: 1001
EOF

# Install latest version of GitLab Runner.
sudo apt-get update
sudo apt-get install -y \
  gitlab-runner

# Register GitLab Runner using configuration file.
# https://docs.gitlab.com/runner/register/index.html
sudo cp ./gitlab-ci/config.toml /etc/gitlab-runner/config.toml

# Make sure OverlayFS driver is loaded on boot.
# https://docs.gitlab.com/ce/ci/docker/using_docker_build.html#use-driver-for-every-project
sudo sh -c \
  'grep -q "overlay" /etc/modules \
  || echo "\n# Make sure OverlayFS driver is loaded on boot.\noverlay" >> /etc/modules'
