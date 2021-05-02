% Plot battery voltage
bat=[];
for i=1:length(j)
  p=j(i).payload;
  if isfield(p,'object') && isfield(p.object,'battery_status')
    if ischar(p.object.battery_voltage)
      bat=[bat,struct('deviceName',p.deviceName,'battery_status',p.object.battery_status,'battery_voltage',str2double(p.object.battery_voltage(1:end-1)))];
    else
      bat=[bat,struct('deviceName',p.deviceName,'battery_status',p.object.battery_status,'battery_voltage',p.object.battery_voltage)];
    end
  elseif isfield(p,'batteryLevel')
    bat=[bat,struct('deviceName',[p.deviceName,'Stat'],'battery_status',p.batteryLevelUnavailable,'battery_voltage',p.batteryLevel)];
  end
end
setfig('battery');clf;
udev=unique({bat.deviceName});
leg1={}; leg2={};
for i=1:length(udev)
  sel=strcmp(udev{i},{bat.deviceName}) & [bat.battery_voltage]>0;
  if strcmp(udev{i}(end-3:end),'Stat') && ~strncmp(udev{i},'RAK',2)
    subplot(211);
    leg1{end+1}=udev{i};
  else
    subplot(212);
    leg2{end+1}=udev{i};
  end
  plot(find(sel),[bat(sel).battery_voltage]);
  hold on;
end
subplot(211);
ylabel('Percent');
legend(leg1,'location','best');
c=axis; c(3:4)=[0,100];axis(c);
ax=gca;
subplot(212);
ylabel('Voltage');
legend(leg2,'location','best');
c=axis; c(3:4)=[3,5];axis(c);
ax(2)=gca;
linkaxes(ax,'x');