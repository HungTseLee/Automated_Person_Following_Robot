#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/hungtselee/robot_ws/src/rbx1/rbx1_nav"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/hungtselee/robot_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/hungtselee/robot_ws/install/lib/python3/dist-packages:/home/hungtselee/robot_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/hungtselee/robot_ws/build" \
    "/home/hungtselee/anaconda3/bin/python" \
    "/home/hungtselee/robot_ws/src/rbx1/rbx1_nav/setup.py" \
    build --build-base "/home/hungtselee/robot_ws/build/rbx1/rbx1_nav" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/hungtselee/robot_ws/install" --install-scripts="/home/hungtselee/robot_ws/install/bin"
