#!/bin/sh
/home/dji/Desktop/a_miaosuan_v6.3.1_new_watcher/build/run
export LD_LIBRARY_PATH=/home/dji/Desktop/a_miaosuan_v6.3.1_new_watcher/build/run
while true; do
        server=ps aux | grep CenterServer_d | grep -v grep
        if [ ! "$server" ]; then
            /home/dji/Desktop/a_miaosuan_v6.3.1_new_watcher/build/run 
	    echo "restart"
            sleep 10
        fi
        sleep 5
done
