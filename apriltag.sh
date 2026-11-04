#!/bin/bash
case "$1" in
  up)
    docker start april_tag_serive
    ;;
  down)
    docker stop april_tag_serive
    ;;
  *)
    echo "Usage: $0 {up|down}"
    exit 1
    ;;
esac
