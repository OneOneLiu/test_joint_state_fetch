xhost local:root
XAUTH=/tmp/.docker.xauth
docker run --rm -it \
    --name=ros_humble_container \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="/home/$USER/humble_ws/test_joint_state_fetch:/humble_ws/src/test_joint_state_fetch" \
    --volume="/dev/bus/usb:/dev/bus/usb" \
    --volume="/tmp/.docker.xauth:/tmp/.docker.xauth:rw" \
    --net=host \
    --privileged \
    ros_humble_image \
    bash

echo "Done."
