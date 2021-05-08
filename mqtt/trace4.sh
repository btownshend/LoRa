/usr/local/homebrew/bin/mosquitto_sub -h 192.168.0.246 -t 'application/2/device/8cf9587200002b9a/rx/#' -F %J  | jq --unbuffered  | grep 'rssi\|deviceName\|loRaSNR\|latitude\|longitude\|"dr"'
