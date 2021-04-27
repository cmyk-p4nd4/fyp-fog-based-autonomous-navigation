#!/bin/bash

num=1
while :
do
  read -rsn4 -t 15 word
  if [[ $word = 'kill' ]]; then
    echo "Script Terminated"
    break
  elif [[ $word = 'incr' ]]; then
    num=`expr $num + 1`
    echo "num=$num"
  elif [[ $word = 'desc' ]]; then
    num=`expr $num - 1`
    echo "num=$num"
  elif [[ $word = 'save' ]] || [[ -z "${word}" ]]; then
    cd maps && rosrun map_server map_saver -f map_${num} && cd ..
    echo "Action Completed"
    #file_count = $(ls -lR maps/*.pgm | wc -l)
  fi
done
