function plotbrc()
% Plot the background map
global brc
if ~exist('brc','var') || isempty(brc)
  brc=kmz2struct('BRC.kmz');
end
sel=find(~strcmp({brc.Geometry},'Point'));

latlim=[min([brc(sel).Lat]),max([brc(sel).Lat])];
lonlim=[min([brc(sel).Lon]),max([brc(sel).Lon])];
%setfig('brc');clf;
hold on;
for i=1:length(sel)
  z=brc(sel(i));
  if strcmp(z.Geometry,'Polygon')
    plot(z.Lon([1:end,1]),z.Lat([1:end,1]),'Color',z.Color);
  else
    plot(z.Lon,z.Lat,'Color',z.Color);
  end
  t=z.Name;
  if ~isempty(strfind(t,'radius circle'))
    t=t(1);
  end
  text(z.Lon(end),z.Lat(end),t);
end
plot(-119.1680952,40.8059733,'or');
axis([-119.2 -119.15   40.793   40.828]);
daspect([1,cosd(mean(latlim)),1])
pbaspect([1,cosd(mean(latlim)),1])
end
