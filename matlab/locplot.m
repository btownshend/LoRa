% Plot location voltage

% Get gateway avg position
sel=arrayfun(@(z) isfield(z.payload,'rxInfo'), j);
gwlat=arrayfun(@(z) z.payload.rxInfo.location.latitude, j(sel));
gwlong=arrayfun(@(z) z.payload.rxInfo.location.longitude, j(sel));

loc=[];
for i=1:length(j)
  p=j(i).payload;
  if isfield(p,'object') && isfield(p.object,'latitude')
    if isfield(p.object,'hdop')
      hdop=p.object.hdop;
    else
      hdop=nan;
    end
    if ischar(p.object.latitude)
      loc=[loc,struct('deviceName',p.deviceName,'latitude',str2double(p.object.latitude(1:end-2)),'longitude',str2double(p.object.longitude(1:end-2)))];
    else
      loc=[loc,struct('deviceName',p.deviceName,'hdop',hdop,'latitude',p.object.latitude,'longitude',p.object.longitude)];
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

nexttile;
for i=1:length(udev)
  sel1=strcmp(udev{i},{loc.deviceName});
  sel=find(sel1 & ([loc.hdop]<1.2 | isnan([loc.hdop])));
  sel=sel(1:ceil(length(sel)/50):end);
  fprintf('%s keeping %d/%d GPS fixes\n', udev{i}, length(sel), sum(sel1));
  h=plot([loc(sel).longitude]+rand(1,length(sel))*.0001-.00005,[loc(sel).latitude]+rand(1,length(sel))*.0001-.00005,'-');
  hold on;
  plot(meanPos(i,2),meanPos(i,1),'o','MarkerSize',15,'LineWidth',2,'HandleVisibility','off','Color',get(h,'Color'));
end
plot(trimmean(gwlong,50),trimmean(gwlat,50),'ok','Markersize',15,'LineWidth',1,'HandleVisibility','off');
legend(udev,'location','best');
axis equal