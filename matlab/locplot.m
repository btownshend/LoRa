% Plot location voltage
loc=[];
for i=1:length(j)
  p=j(i).payload;
  if isfield(p,'object') && isfield(p.object,'latitude')
    if ischar(p.object.latitude)
      loc=[loc,struct('deviceName',p.deviceName,'latitude',str2double(p.object.latitude(1:end-2)),'longitude',str2double(p.object.longitude(1:end-2)))];
    else
      loc=[loc,struct('deviceName',p.deviceName,'latitude',p.object.latitude,'longitude',p.object.longitude)];
    end
  end
end
bad=[loc.latitude]<37 | [loc.longitude]>-122;
loc=loc(~bad);
udev=unique({loc.deviceName});

setfig('location');clf;
tiledlayout('flow');
a=[];
for i=1:length(udev)
  nexttile;
  sel=strcmp(udev{i},{loc.deviceName});
  plot(find(sel),[loc(sel).latitude],'-');
  axis([1,length(loc),min([loc.latitude]),max([loc.latitude])]);
  ylabel('Latitude');
  a(i,1)=gca;
  hold on;
  yyaxis right
  plot(find(sel),[loc(sel).longitude],'-');
  ylabel('Longitude');
  axis([1,length(loc),min([loc.longitude]),max([loc.longitude])]);
  a(i,2)=gca;

  title(udev{i});
end
linkaxes(a(:,1),'y');
linkaxes(a(:,2),'y');
linkaxes(a(:),'x');

nexttile;
for i=1:length(udev)
  sel=strcmp(udev{i},{loc.deviceName});
  plot([loc(sel).latitude],[loc(sel).longitude],'-');
  hold on;
end
axis ij;

  
