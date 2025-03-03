```
docker run -it --rm --privileged --device=/dev/bus/usb:/dev/bus/usb \
    -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev -v ~/dev:/dev_project \
    --name my_usb_container_instance my_usb_container

```


```
docker run -it --rm \
    --privileged \
    --device=/dev/bus/usb:/dev/bus/usb \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    --name my_usb_container_instance \
    my_usb_container

```