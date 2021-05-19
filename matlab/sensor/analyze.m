% Analyze data in v
setfig('sensor');clf;
tiledlayout('flow');
for i=1:length(v)
  nexttile;
  plot(v(i).sensor);
  hold on;
  range=mod(v(i).range-1,360)+1;
  plot(range,v(i).sensor(range),'r');
  plot([0,360],v(i).offset*[1,1],':');
  ax=axis;
  plot(mod(v(i).peakpos-1,360)+1*[1,1],ax(3:4),':');
  axis tight;
  xlabel('Angle (deg)');
  ylabel('Sensor value');
  title(sprintf('Peak pos=%d',v(i).peakpos));
end

  