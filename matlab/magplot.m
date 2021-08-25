setfig('magplot');clf;
tiledlayout('flow');
for i=1:length(topic)
  t=topic(i);
  mag=[];
  for j=1:length(t.payload)
    p=t.payload(j);
    if isfield(p,'object')
      obj=p.object;
      if isfield(obj,'magnetometer_x')
        mag=[mag;obj.magnetometer_x,obj.magnetometer_y,obj.magnetometer_z];
      end
    end
  end
  if ~isempty(mag)
    nexttile;
    plot3(mag(:,1),mag(:,2),mag(:,3),'.');
    title(t.topic);
    axis equal;
  end
end
