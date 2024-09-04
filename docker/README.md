# Setup Docker

> This repository is based on `ROS-humble` 

## Build Docker Image
In current path, run the following command to build the docker Image with the name `ros_humble_image`.

```bash
bash build.bash
```

> **Note:**
> - You can change the name to whatever you like, just remember to update the new name in the [startup file](humble.bash).

## Start Docker container
In current path, run the following command to start the docker container with the name `ros_humble_container`:
```bash
bash humble.bash
```