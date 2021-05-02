function plotvalue(topic,field,usemarkers)
t0=now+7/24;
leg={};
sym='ox+*sdv^<>ph';symcnt=1;
if nargin<4
  usemarkers=false;
end
if nargin<3
  units='';
end
 
for i=1:length(topic)
  val=getnumfield(topic(i).payload,field);
  if any(isfinite(val))
    t=gettime(topic(i).payload)-t0;
    if usemarkers
      plot(t*24*60,val,sym(symcnt));
      symcnt=symcnt+1;
    else
      plot(t*24*60,val,'-');
    end      
    hold on;
    leg{end+1}=gettopicname(topic(i));
  end
end
if length(leg)>1
  legend(leg);
  title(field);
else
  title([leg{1},' ',field]);
end
xlabel('Time (min)');
ylabel(field);
end
