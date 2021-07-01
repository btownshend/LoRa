% Plot rx info

setfig('snr');clf;
tiledlayout('flow');
nexttile;
plotvalue(topic,'rxInfo.loRaSNR',true);
ylabel('SNR (dB)');
ax=gca;
nexttile;
plotvalue(topic,'rxInfo.rssi',false);
ylabel('RSSI (dBm)');
ax(end+1)=gca;
nexttile;
plotvalue(topic,'txInfo.dr',false);
ylabel('DR');
ax(end+1)=gca;
linkaxes(ax,'x');
nexttile;
crossplot(topic,'rxInfo.rssi','rxInfo.loRaSNR');
