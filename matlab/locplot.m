% Plot location

% Get gateway avg position
sel=arrayfun(@(z) isfield(z.payload,'rxInfo'), j);
gwlat=arrayfun(@(z) z.payload.rxInfo.location.latitude, j(sel));
gwlong=arrayfun(@(z) z.payload.rxInfo.location.longitude, j(sel));

loc=[];
t0=now()+7/24;
for i=1:length(j)
  p=j(i).payload;
  if isfield(p,'object') && isfield(p.object,'latitude')
    rssi=p.rxInfo.rssi;
    if isfield(p.object,'hdop')
      hdop=p.object.hdop;
    else
      hdop=nan;
    end
    if ischar(p.object.latitude)
      loc=[loc,struct('time',gettime(p)-t0,'deviceName',p.deviceName,'rssi',rssi,'latitude',str2double(p.object.latitude(1:end-2)),'longitude',str2double(p.object.longitude(1:end-2)))];
    else
      loc=[loc,struct('time',gettime(p)-t0,'deviceName',p.deviceName,'hdop',hdop,'rssi',rssi,'latitude',p.object.latitude,'longitude',p.object.longitude)];
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
  plot([loc(sel).time]*24*60,[loc(sel).latitude],'-');
  %axis([1,length(loc),min([loc.latitude]),max([loc.latitude])]);
  ylabel('Latitude');
  a(i,1)=gca;
  hold on;
  yyaxis right
  plot([loc(sel).time]*24*60,[loc(sel).longitude],'-');
  ylabel('Longitude');
  %axis([1,length(loc),min([loc.longitude]),max([loc.longitude])]);
  a(i,2)=gca;
  title(udev{i});
  % Compute home position
  lat=[loc(sel).latitude];
  long=[loc(sel).longitude];
  medianLat=nanmedian(lat);
  medianLong=nanmedian(long);
  sel2= abs(lat-medianLat<.0002) & abs(long-medianLong<.0002);
  meanLat=nanmean(lat(sel2));
  meanLong=nanmean(long(sel2));
  fprintf('%-10.10s: Median: (%.4f,%.4f), Mean over %d/%d: (%f,%f)\n', udev{i}, medianLat, medianLong, sum(sel2),sum(sel), meanLat, meanLong);
  meanPos(i,:)=[meanLat,meanLong];
end
linkaxes(a(:,1),'y');
linkaxes(a(:,2),'y');
linkaxes(a(:),'x');

for i=1:length(udev)
  nexttile;
  if any([loc.latitude]>40)
    plotbrc; axis auto;
  end
  sel1=strcmp(udev{i},{loc.deviceName});
  sel=find(sel1 & ([loc.hdop]<=1.3 | isnan([loc.hdop])));
  ds=round(length(sel)/50);
  ds=1;
  lat=decimate([loc(sel).latitude],ds,'FIR');
  long=decimate([loc(sel).longitude],ds,'FIR');
  fprintf('%s keeping %d/%d GPS fixes downsampled by %f\n', udev{i}, length(sel), sum(sel1),ds);
  %  h=plot(medfilt1([loc(sel).longitude],ds,'truncate')+rand(1,length(sel))*.0001-.00005,medfilt1([loc(sel).latitude],ds,'truncate')+rand(1,length(sel))*.0001-.00005,'-');
  h=plot(long,lat,'-o');
  hold on;
  alllat=[loc(sel1).latitude];
  alllong=[loc(sel1).longitude];
  plot(alllong,alllat,'.r');
  plot(meanPos(i,2),meanPos(i,1),'o','MarkerSize',15,'LineWidth',2,'HandleVisibility','off','Color',get(h,'Color'));
  plot(trimmean(gwlong,50),trimmean(gwlat,50),'ok','Markersize',15,'LineWidth',1,'HandleVisibility','off');
  title(udev{i});
end
%legend(udev,'location','best');
%axis equal

% RSSI vs position
setfig('rssi vs pos');clf;
tiledlayout('flow');
for i=1:length(udev)
  nexttile;
  sel1=strcmp(udev{i},{loc.deviceName});
  sel=find(sel1 & ([loc.hdop]<=1.3 | isnan([loc.hdop])));
  lat=[loc(sel).latitude];
  long=[loc(sel).longitude];
  rssi=[loc(sel).rssi];
  ngrid=20;
  [x,y]=meshgrid(linspace(min(long),max(long),ngrid),linspace(min(lat),max(lat),ngrid));
  z=griddata(long,lat,rssi,x,y,'linear');

  C=contour(x,y,z);
  clabel(C);
  xlabel('Longitude');
  ylabel('Latitude');
  hold on;
  plot(long,lat,'or');
  title(sprintf('Position vs RSSI for %s',udev{i}));
end

