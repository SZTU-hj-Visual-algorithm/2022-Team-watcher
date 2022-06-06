#!/bin/sh
/home/dji/Desktop/a_miaosuan_v6.3.1_new/build3/run
export LD_LIBRARY_PATH=/home/dji/Desktop/a_miaosuan_v6.3.1_new/build3/run
while true; do
        server=ps aux | grep CenterServer_d | grep -v grep
        if [ ! "$server" ]; then
            /home/dji/Desktop/a_miaosuan_v6.3.1_new/build3/run 
	    echo "restart"
            sleep 1
        fi
        sleep 1
done
