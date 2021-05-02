maxdata=3000;
fd=fopen('log.txt','r');
data=fread(fd);
fclose(fd);
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
