function crossplot(topic,field1,field2,usemarkers)
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
  val1=getnumfield(topic(i).payload,field1);
  val2=getnumfield(topic(i).payload,field2);
  if any(isfinite(val1)&isfinite(val2))
    % Add noise
    val1=val1+(rand(size(val1))-0.5)*0.7*min(diff(sort(unique(val1))));
    val2=val2+(rand(size(val2))-0.5)*0.7*min(diff(sort(unique(val2))));
    if usemarkers
      plot(val1,val2,sym(symcnt));
      symcnt=symcnt+1;
    else
      plot(val1,val2,'.');
    end      
    hold on;
    leg{end+1}=gettopicname(topic(i));
  end
end
if length(leg)>1
  legend(leg);
  title(sprintf('%s vs %s',field1,field2));
else
  title(sprintf('%s vs %s - %s',field1,field2,leg{1}));
end
xlabel(field1);
ylabel(field2);
end
