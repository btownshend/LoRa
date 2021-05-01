% Plot rx info
rx=[];
for i=1:length(j)
  p=j(i).payload;
  if isfield(p,'rxInfo')
    rx=[rx,struct('deviceName',p.deviceName,'rssi',p.rxInfo.rssi,'snr',p.rxInfo.loRaSNR)];
  end
end
udev=unique({rx.deviceName});

setfig('snr');clf;
tiledlayout('flow');
nexttile;
for i=1:length(udev)
  sel=strcmp(udev{i},{rx.deviceName});
  plot(find(sel),[rx(sel).snr],'-');
  hold on;
end
legend(udev);
title('SNR');
a=gca;

nexttile;
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
