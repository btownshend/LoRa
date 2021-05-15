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
nexttile;
crossplot(topic,'rxInfo.rssi','rxInfo.loRaSNR');
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
linkaxes(a,'x');

nexttile;
leg={};
for i=1:length(topic)
  rssi=getnumfield(topic(i).payload,'rxInfo','rssi');
  snr=getnumfield(topic(i).payload,'rxInfo','loRaSNR');
  if any(isfinite(rssi) & isfinite(snr))
    plot(rssi,snr,'o');
    hold on;
    leg{end+1}=gettopicname(topic(i));
  end
end
legend(leg);
xlabel('RSSI (dBm)');
ylabel('SNR (dB)');
title('SNR vs RSSI');

