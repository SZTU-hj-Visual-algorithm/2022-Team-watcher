#!/bin/sh
/home/nvidia/Desktop/a_miaosuan_v6.3.1_height_watcher/build/run
export LD_LIBRARY_PATH=/home/nvidia/Desktop/a_miaosuan_v6.3.1_height_watcher/build/run
while true; do
        server=ps aux | grep CenterServer_d | grep -v grep
        if [ ! "$server" ]; then
            /home/nvidia/Desktop/a_miaosuan_v6.3.1_height_watche/build/run 
	    echo "restart"
            sleep 10
        fi
        sleep 5
done
