function tm=gettime(payload)
% Retrieve rx time from a payload
  tm=nan(size(payload));
  tstr=getcharfield(payload,'rxInfo.time');
  for i=1:length(tstr)
    if ~isempty(tstr{i})
      tm(i)=datenum(tstr{i}(1:23),'yyyy-mm-ddTHH:MM:SS.FFF');
    end
  end
end
