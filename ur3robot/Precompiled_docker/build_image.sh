#!/bin/bash

tar -cvzf ros_code_archive.tar.gz ../ros_code
docker image build -t ur3robot_precompiled_docker_image .
