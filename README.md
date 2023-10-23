# my_ros2_docker_image

This template repository should give a good overview on how to build your own ROS2 development instance and produce a new hosted docker image out of it.

## Only needed for multiplatform images
Following this guide you will also be able to do multiplatform builds with docker buildx.
To build multiplatform images need binfmt dependencies for arm64 architecture as requirement (The resulting image works on PCs as well as Jetson Nano, Raspberry Pi, Apple Silicon Devices).
The sections for multiplatform builds are commented out in the configuration files so you also need to uncomment these lines.
### Install requirements
```
docker run --privileged --rm tonistiigi/binfmt --install all
```
### Uninstall
```
docker run --privileged --rm myuser/binfmt
```

## What you will want to change in your repository:
### my.repos and my_extended.repos:
These are yaml files containing the repositories you may want to include into your final ros2 workspace.
For a multi-stage build for instance, I define my base image repositories in "my.repos" and the extended image repositories in "my_extended.repos"

## To build the image for my_example (and push it to your container registry):
```
docker buildx bake my_example --load # to load into local container registry
```
```
docker buildx bake my_example --push # to push into specified remote container registry
```
## To run my_example:
### Run my_example just once to see if everything's working:

### Run my_example in background
```
docker compose up -d my_example
```
### Run my_example in foreground
```
docker compose up my_example
```
### Execute shell inside my_example
#### If container is stopped
```
docker compose run my_example bash
```
#### If container is already running
```
docker compose exec my_example bash
```
### Execute example_program inside my_example
```
docker compose run my_example example_program
```
