HOST=192.168.230.1
#scp  -r pi@192.168.0.189:~/LoRa/mqtt/logs .
rsync -av -r pi@${HOST}:~/LoRa/mqtt/logs .
