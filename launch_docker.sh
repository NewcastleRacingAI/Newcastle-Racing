xhost +local:root  # Allow local root (only needs to be run once per session)

docker run -it \
  --net=host \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  --env XAUTHORITY=/root/.Xauthority \
  --privileged \
  nufs \
  bash

