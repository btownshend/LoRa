maxdata=20000;
fd=fopen('log.txt','r');
data=fread(fd);
fclose(fd);
s=strsplit(char(data)');
fprintf('Loaded %d lines\n', length(s));
j=[];jcnt=0;
for i=max(1,length(s)-maxdata):length(s)
  nb=find(s{i}~=' ' & s{i}~=0,1);
  if ~isempty(nb)
    st=s{i}(nb:end);
    jcnt=jcnt+1;
    if jcnt==1
      % Pre-allocate
      j=jsondecode(st);
      j(length(s))=j;
    else
      j(jcnt)=jsondecode(st);
    end
  end
end
j=j(1:jcnt);
fprintf('Formed %d JSON expressions\n', length(j));
