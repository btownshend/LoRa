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
ax(2)=gca;
linkaxes(ax,'x');
return;

leg={};
sym='ox+*sdv^<>ph';symcnt=1;
for i=1:length(topic)
  snr=getnumfield(topic(i).payload,'rxInfo','loRaSNR');
  if any(isfinite(snr))
    t=gettime(topic(i).payload)-t0;
    plot(t*24*60,snr,sym(symcnt));
    symcnt=symcnt+1;
    hold on;
    leg{end+1}=gettopicname(topic(i));
  end
end
legend(leg);
xlabel('Time (min)');
ylabel('SNR (dB)');
title('SNR');
a=gca;

nexttile;
leg={};
for i=1:length(topic)
  rssi=getnumfield(topic(i).payload,'rxInfo','rssi');
  if any(isfinite(rssi))
    t=gettime(topic(i).payload)-t0;
    plot(t*24*60,rssi,'-');
    hold on;
    leg{end+1}=gettopicname(topic(i));
  end
end
legend(leg);
xlabel('Time (min)');
ylabel('RSSI (dBm)');
title('RSSI');
a(end+1)=gca;

return;

for i=1:length(udev)
  sel=strcmp(udev{i},{rx.deviceName});
  plot(find(sel),[rx(sel).rssi]);
  hold on;
end
title('RSSI');
a(2)=gca;

linkaxes(a,'x');

nexttile;
for i=1:length(udev)
  sel=strcmp(udev{i},{rx.deviceName});
  snr=[rx(sel).snr]; snr=snr+(rand(size(snr))*2-1)*0.25;
  rssi=[rx(sel).rssi]; rssi=rssi+(rand(size(rssi))*2-1)*0.5;
  plot(rssi,snr,'.');
  hold on;
end
title('RSSI vs SNR');
ylabel('SNR');
xlabel('RSSI');
