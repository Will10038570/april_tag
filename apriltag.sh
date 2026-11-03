#!/bin/bash
case "$1" in
  up)
    docker start april_tag_service
    ;;
  down)
    docker stop april_tag_service
    ;;
  *)
    echo "Usage: $0 {up|down}"
    exit 1
    ;;
esac
