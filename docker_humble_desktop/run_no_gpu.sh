# # If not working, first do: sudo rm -rf /tmp/.docker.xauth
# # If still not working, try running the script as root.

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")

docker run -it \
    --name hd_humble_desktop \
    --rm \
    --privileged \
    --net=host \
    -e DISPLAY=unix$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $XAUTH:$XAUTH \
    -v /run/user/1000/at-spi:/run/user/1000/at-spi \
    -v /dev:/dev \
    -v $parent_dir:/home/xplore/dev_ws/src \
    -v hd_humble_desktop_home_volume:/home/xplore \
    ghcr.io/epflxplore/hd:humble-desktop \
    /bin/bash -c "sudo chown -R $USERNAME:$USERNAME /home/$USERNAME; /bin/bash "


!/bin/bash

# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# If still not working, try running the script as root.

# XAUTH=/tmp/.docker.xauth

# echo "Preparing Xauthority data..."
# xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
# if [ ! -f $XAUTH ]; then
#     if [ ! -z "$xauth_list" ]; then
#         echo $xauth_list | xauth -f $XAUTH nmerge -
#     else
#         touch $XAUTH
#     fi
#     chmod a+r $XAUTH
# fi

# echo "Done."
# echo ""
# echo "Verifying file contents:"
# file $XAUTH
# echo "--> It should say \"X11 Xauthority data\"."
# echo ""
# echo "Permissions:"
# ls -FAlh $XAUTH
# echo ""
# echo "Running docker..."

# # Get the current working directory
# current_dir=$(pwd)

# # Use dirname to get the parent directory
# parent_dir=$(dirname "$current_dir")

# # Set the USERNAME variable if not set
# USERNAME=${USERNAME:-$(whoami)}

# docker run -it \
#     --name hd_humble_desktop \
#     --rm \
#     --privileged \
#     --net=host \
#     -e DISPLAY=unix$DISPLAY \
#     -e QT_X11_NO_MITSHM=1 \
#     -e XAUTHORITY=$XAUTH \
#     -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
#     -v $XAUTH:$XAUTH \
#     -v /run/user/1000/at-spi:/run/user/1000/at-spi \
#     -v /dev:/dev \
#     -v $parent_dir:/home/xplore/dev_ws/src \
#     -v hd_humble_desktop_home_volume:/home/xplore \
#     ghcr.io/epflxplore/hd:humble-desktop \
#     /bin/bash -c "sudo chown -R $USERNAME:$USERNAME /home/$USERNAME; \
#                   source /opt/ros/humble/setup.bash; \
#                   source /home/xplore/dev_ws/install/setup.bash; \
#                   sudo -i; \
#                   cd /home/xplore/dev_ws; \
#                   /bin/bash"

