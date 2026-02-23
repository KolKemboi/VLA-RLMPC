docker run -it --rm --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name "openvla" -v $(pwd)/models:/workspace/models openvla-cpu
