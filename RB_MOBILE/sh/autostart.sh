#!/bin/bash

cd ~/code/build-SLAMNAV-Desktop-Release
./SLAMNAV

while [ 1 ]
do
	pid=$(ps -ef | grep "SLAMNAV" | grep -v 'grep' | awk '{print $2}')
	if [ -z "$pid" ];
	 then
    	~/code/build-SLAMNAV-Desktop-Release/SLAMNAV
	else
    	kill -9 $pid
    	~/code/build-SLAMNAV-Desktop-Release/SLAMNAV
	fi
	
	done
exit 0














































