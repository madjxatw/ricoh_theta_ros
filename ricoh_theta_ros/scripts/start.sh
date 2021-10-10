#!/bin/bash

camctl=ricoh

_chkmod()
{
  lsmod | grep -q "$1" >/dev/null 2>&1
}

_chkcmd()
{
  command -v "$1" >/dev/null 2>&1
}

_cmderr()
{
  printf "$1: command not found\n" 1>&2
  exit 1
}

_moderr()
{
  printf "$1: module not loaded\n" 1>&2
  exit 1
}

# prechecks
if ! _chkmod 'v4l2loopback'; then
  _moderr 'v4l2loopback'
fi

if ! _chkcmd 'gst_loopback'; then
  _cmderr 'gst_loopback'
fi

if ! _chkcmd 'ptpcam'; then
  _cmderr 'ptpcam'
fi

if ! _chkcmd 'ricoh'; then
  printf "ricoh: command not found. You have to use ptpcam directly.\n"
  camctl=ptpcam
fi

# wake the camera up and ensure the live streaming mode
case "${camctl}" in
  'ricoh')
    ricoh wake && ricoh mode live
    ;;
  'ptpcam')
    ptpcam --set-property=0xd80e 0 && ptpcam --set-property=0x5013 --val=0x8005
    ;;
  *)
    exit 1
    ;;
esac

# put it in background to avoid blocking
gst_loopback --format 2K &

# launch cv_camera_node
roslaunch ricoh_theta_ros start.launch device_id:=2

# Killing the parent process does not kill the background child processes (e.g.
# gst_loopback in this case), hence we use the bash built-in `trap` to kill all
# the background child processes when this script is terminated.
trap 'kill $(jobs -p)' EXIT
