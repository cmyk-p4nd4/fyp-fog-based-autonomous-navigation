#!/bin/bash
while :; do
  sleep 1s
  ./scripts/mapMerge.py maps/map_1.pgm maps/map_2.pgm maps/map_out.pgm
  echo "Action Completed"
  sleep 1s
  ./scripts/trigger_server.py
done
