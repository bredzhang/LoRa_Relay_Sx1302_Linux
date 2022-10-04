#!/bin/sh
while true;do
    count=`ps -A|grep lora_pkt_fwd|grep -v grep`
    if [ "$?" != "0" ];then
	./lora_pkt_fwd &
    fi
    sleep 5
    done
