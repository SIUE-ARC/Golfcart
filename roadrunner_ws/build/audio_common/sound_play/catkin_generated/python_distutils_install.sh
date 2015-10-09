#!/bin/sh -x

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

cd "/home/roadrunner/roadrunner_ws/src/audio_common/sound_play"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/roadrunner/roadrunner_ws/install/lib/python2.7/dist-packages:/home/roadrunner/roadrunner_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/roadrunner/roadrunner_ws/build" \
    "/usr/bin/python" \
    "/home/roadrunner/roadrunner_ws/src/audio_common/sound_play/setup.py" \
    build --build-base "/home/roadrunner/roadrunner_ws/build/audio_common/sound_play" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/roadrunner/roadrunner_ws/install" --install-scripts="/home/roadrunner/roadrunner_ws/install/bin"
