#!/bin/bash

read -p "How many vehicle client containers do you want to start? " count

read -p "Use ML (Y/n)? " use_ml

read -p "Rebuild containers (Y/n)? " rebuild

if [[ "$rebuild" = "Y" || "$rebuild" = "y" ]]; then
    echo "Rebuilding container image"
    sudo docker build -f Dockerfile -t vehicle-sim:latest .
fi

echo "Starting $count Vehicle Client Containers..."

sudo docker stop $(sudo docker ps -a -q)
sudo docker rm $(sudo docker ps -a -q)

for ((i=0; i<$count; i++))
do
    if [[ "$use_ml" = "Y" || "$use_ml" = "y" ]]; then
        sudo nvidia-docker run --gpus all -d --network=host -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY vehicle-sim --apply_ml 
        # sudo docker run -d --network=host vehicle-sim
    else 
        sudo nvidia-docker run --gpus all -d --network=host -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY vehicle-sim
    fi    
done

sudo docker container ls