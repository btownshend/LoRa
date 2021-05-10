setfig('acc');clf;
tiledlayout('flow');
nexttile;
t=d(:,1)/1000;
t=t-t(1);
acc=d(:,2:4)/2048;
mag=sqrt(sum(acc.^2,2));
plot(t,acc);
hold on;
plot(t,mag);
legend('x','y','z','mag');
xlabel('Time (s)');
ylabel('Accel (g)');
nexttile;
orient=atan2(sqrt(sum(acc(:,1:2).^2,2)),acc(:,3))*180/pi;
still=abs(mag-1)<0.05;
still(2:end-1)=still(2:end-1)&still(1:end-2)&still(3:end);
plot(t(still),orient(still),'o-');
xlabel('Time (s)');
ylabel('Orientation (deg)');
%clear s;

nexttile;
plot(t,mag-1);
hold on;
tapthresh=0.5;
holdoff=0.5;
taps=[];
nstill=0;
external=abs(mag-1);
for i=1:length(external)
  if external(i)>tapthresh && nstill>5 && (length(taps)==0 || (t(i)-t(taps(end)) > holdoff))
    taps(end+1)=i;
    fprintf('i=%d, T=%.1f, ext=%.1f, nstill=%d, tilt=%.0f\n', i, t(i), external(i), nstill, orient(i-1));
  elseif external(i)>tapthresh
    fprintf('Ignore %d, ext=%.1f, nstill=%d\n', i, external(i), nstill);
  end
  if external(i)>0.1
    nstill=0;
  else
    nstill=nstill+1;
  end
end

plot(t(taps),ones(size(taps)),'x');
xlabel('Time (s)');
ylabel('Accel (g)');

