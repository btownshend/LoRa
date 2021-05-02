function s=gettopicname(topic)
  tsplit=strsplit(topic.topic,'/');
  if isfield(topic.payload,'deviceName') && length(unique({topic.payload.deviceName}))==1
    s=[topic.payload(1).deviceName,'/',tsplit{end}];
  else
    s=strjoin(tsplit(end-1:end),'/');
  end
end


  