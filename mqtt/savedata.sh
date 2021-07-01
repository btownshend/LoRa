#/bin/bash
dt=$(date "+%Y%m%d-%H%M%S")
logfile=MQTT+${dt}.log
echo Saving application topic in $logfile
/usr/local/homebrew/bin/mosquitto_sub -h 192.168.0.189 -t "application/#" -F %J > $logfile


