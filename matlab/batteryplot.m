% Plot battery voltage
bat=[];
for i=1:length(j)
  p=j(i).payload;
  if isfield(p,'object') && isfield(p.object,'battery_status')
    bat=[bat,struct('deviceName',p.deviceName,'battery_status',p.object.battery_status,'battery_voltage',str2double(p.object.battery_voltage(1:end-1)))];
  elseif isfield(p,'batteryLevel')
    bat=[bat,struct('deviceName',p.deviceName,'battery_status',p.batteryLevelUnavailable,'battery_voltage',p.batteryLevel)];
  end
end
setfig('battery');clf;
udev=unique({bat.deviceName});
for i=1:length(udev)
  sel=strcmp(udev{i},{bat.deviceName}) & [bat.battery_voltage]>1;
  plot(find(sel),[bat(sel).battery_voltage]);
  hold on;
end
legend(udev);
       