# my_ros2_docker_image

Following this you will be able to do multiplatform builds with docker buildx.
For the build with docker buildx you will need binfmt dependencies for arm64 architecture, as we are doing a multiplatform build (Works on PCs as well as Jetson Nano, Raspberry Pi, Apple Silicon Devices).

## What you will want to change:
### my.repos and my_extended.repos:
These are yaml files containing the repositories you may want to include into your final ros2 workspace.
For a multi-stage build for instance, I define my base image repositories in "my.repos" and the extended image repositories in "my_extended.repos"

## To build the image for example_service (and push it to your container registry):
docker buildx bake example_service --load
docker buildx bake example_service --push

## To run example_service:
docker compose up -d example_service
