#!/bin/bash

while [ 1 ]
do
	pid=$(ps -ef | grep "SLAMNAV" | grep -v 'grep' | awk '{print $2}')
	if [ -z "$pid" ];
	 then
    	~/CODE/2024_orderpicking_code/order_ui_d435f/build-LIFT_PROG_d435f_0702-Desktop-Release/LIFT_PROG_d435f_0702
	else
    	kill -9 $pid
    	~/CODE/2024_orderpicking_code/order_ui_d435f/build-LIFT_PROG_d435f_0702-Desktop-Release/LIFT_PROG_d435f_0702
	fi
done
