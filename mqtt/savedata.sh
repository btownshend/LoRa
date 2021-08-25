#/bin/bash
PATH=${PATH}:/usr/local/homebrew/bin
dt=$(date "+%Y%m%d-%H%M%S")
logfile=MQTT+${dt}.log
echo Saving application topic in $logfile
if [ $(uname) == "Linux" ]
then
  host=127.0.0.1
else
  host=192.168.0.189
fi
echo Listening to $host
# mosquitto_sub -h $host -t "application/#" -F %J > $logfile
mosquitto_sub -h $host -t "application/#" -F %J | rotatelogs -lf logs/MQTT-%Y%m%d-%X.log 3600

