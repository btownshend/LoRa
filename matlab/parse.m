% Plot events from last hours 
function [topic,j]=parse(hours)
  if nargin<1
    hours=24;
  end
  maxdata=3000;
  d=dir('../mqtt/MQTT*.log');
  keep=(now-[d.datenum])*24 <= hours;
  d=d(keep);
  [~,ord]=sort([d.datenum],'asc');
  d=d(ord);
  data=[];
  for i=1:length(d)
    filename=d(i).name;
    path=[d(i).folder,'/',d(i).name];
    fprintf('Loading %s\n',path);
    fd=fopen(path,'r');
    if fd<0
      fprintf('Unable to open %s\n', path);
      return;
    end
    data=[data;fread(fd)];
    fclose(fd);
  end
  s=strsplit(char(data)','\n');
  fprintf('Loaded %d lines\n', length(s));
  j=[];jcnt=0;
  for i=max(1,length(s)-maxdata):length(s)
    nb=find(s{i}~=' ' & s{i}~=0,1);
    if ~isempty(nb) && s{i}(nb)=='{'
      st=s{i}(nb:end);
      jcnt=jcnt+1;
      if jcnt==1
        % Pre-allocate
        j=jsondecode(st);
        j(length(s))=j;
      else
        try
          j(jcnt)=jsondecode(st);
        catch me
          fprintf('Unable to decode line %d\n', i);
          jcnt=jcnt-1;
        end
      end
    else
      fprintf('Ignoring line: %s\n',s{i});
    end
  end
  j=j(1:jcnt);
  fprintf('Formed %d JSON expressions\n', length(j));

  topics=unique({j.topic});
  topic=[];
  for i=1:length(topics)
    topic=[topic,struct('topic',topics{i},'payload',[j(strcmp({j.topic},topics{i})).payload])];
  end
end
