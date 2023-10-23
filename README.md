# my_ros2_docker_image

This template repository should give a good overview on how to build your own ROS2 development instance and produce a new hosted docker image out of it.
For now the steps below are only suited for a existing Ubuntu installation. Windows is for now not supported in this repository.

## Install dependencies needed for working with docker images and tools to make your life easier
Using the official Docker convenience installer script, we will install docker and docker compose:
```
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```
Then add yourself to the docker group so you must not always use sudo when tinkering with docker commands:
```
sudo groupadd docker
sudo usermod -aG docker $USER
```
Update the group changes:
```
newgrp docker
```
Source: https://docs.docker.com/engine/install/ubuntu/#install-using-the-convenience-script

## Simple build
First things first: Let's begin with a short and simple build. This is a short hello, world! example.
### Clone this repository to your local machine
```
git clone https://github.com/bjoernellens1/ros2_docker_template
```
### Build the hello, world! image
```
docker buildx bake hello-world --load # to load into local container registry
```
or
```
docker buildx bake hello-world --push # to push into specified remote container registry
```
### Run the hello, world! image
```
docker compose run hello-world
```

## Advanced build: Custom ROS2 image that includes the Unitree GO1 package.
```
docker buildx bake my_example --load # to load into local container registry
```
or
```
docker buildx bake my_example --push # to push into specified remote container registry
```
## To run my_example:
### Run my_example just once to see if everything's working:
This will launch the Unitree udp high-level control node.
```
docker compose run my_example
```
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
You will find that example_program does not exist. Try using rqt for example:
```
docker compose run my_example rqt
```

## Next steps
### Fork this repository and make it your own
### What you will want to change in your repository:
#### my.repos and my_extended.repos:
These are yaml files containing the repositories you may want to include into your final ros2 workspace.
For a multi-stage build for instance, I define my base image repositories in "my.repos" and the extended image repositories in "my_extended.repos"

## Additional Resources
For further references please visit my WIP repositories for mobile robotic platforms:
- https://github.com/bjoernellens1/ros2_rmp
- https://github.com/bjoernellens1/cps_bot_mini_ws
