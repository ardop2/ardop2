#!/bin/bash

rosdep update
sudo apt-get update
sudo dpkg -R --install deb/
sudo apt-get install -f