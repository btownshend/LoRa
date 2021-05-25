classdef ardserial < handle
  properties
    devname;  
    samples;   % IMU samples (N,9) - gx,gy,gz,ax,ay,az,mx,my,mz - unscaled raw
    curCal;    % Current calibration struct
    newCal;    % Computed calibration struct
    port;
  end
  
  methods
    function obj=ardserial(devname)
      if nargin<1 || isempty(devname)
        devname='';
        devs=serialportlist();
        for i=1:length(devs)
          if strncmp(devs{i},'/dev/cu.usbmodem',16)
            devname=devs{i};
            break;
          end
        end
        if isempty(devname)
          error('No device given and no matching device found');
        end
      end
      obj.devname=devname;
      obj.curCal=struct('b',[0,0,0],'A',eye(3));
      obj.open();
    end

    function open(obj)
      fprintf('Opening %s\n',obj.devname);
      obj.port=serialport(obj.devname,115200);
      if isempty(obj.port)
        error('Failed open of %s',obj.devname);
      end
      fprintf('Opened %s\n', obj.devname);
    end

    function close(obj)
      obj.port=[];
    end
    
    function flush(obj)
      while (obj.port.NumBytesAvailable > 0)
        line=obj.readline();
        fprintf('Flush: %s\n', line);
      end
    end
    
    function data=readline(obj)
      data=readline(obj.port);
      data=char(data);
      while ~isempty(data) && (data(end)==10 || data(end)==13)
        data=data(1:end-1);
      end
    end
    
    function writeline(obj,line)
      if length(line)>99
        error('board only supports line <100 chars');
      end
      obj.port.writeline(line);
    end
    
    function found=waitfor(obj,expect,timeout)
    % Wait until expect string is returned or timeout expires
      termtime=now+timeout/24/60/60;
      while (now<termtime)
        line=obj.readline();
        if ~isempty(strfind(line,expect))
          fprintf('waitfor: %s\n',line);
          found=true;
          return;
        else
          fprintf('Ignore: %s\n',line);
        end
      end
      fprintf('waitfor: %s not found\n',expect);
      found=false;
      return;
    end
    
    function acquire(obj,numsamples)
    % Put unit in IMU test mode and record samples
      if nargin<2
        numsamples=3000;
      end
      
      setfig('Acquisition');clf;
      obj.flush();
      obj.writeline('x');  % In case we're already in IMON mode
      obj.writeline('IMAT');
      if (~obj.waitfor('+++IMAT+++',1))
        return;
      end
      fprintf('In IMAT mode\n');
      obj.samples=[];
      cntr=1;
      while cntr<=numsamples
        line=obj.readline();
        if isempty(line)
          fprintf('No data returned from readline()\n');
          return;
        end
        if strncmp(line,'MAGSET',6)
          v=sscanf(line,'MAGSET %f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
          if length(v)~=12
            fprintf('Parse error: %s\n', line);
          else
            obj.curCal=struct('b',v(1:3)','A',reshape(v(4:12),3,3));
            fprintf('\nCurrent calibration: %s\n',sprintf('%.2f ',v));
          end
          pause(0.1);
        elseif strncmp(line,'Mat:',4)
          v=sscanf(line,'Mat:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d');
          if length(v)~=11
            fprintf('Parse error: %s\n', line);
          else
            obj.samples=[obj.samples,struct('t',v(1),'g',v(2:4),'a',v(5:7),'m',v(8:10),'step',v(11))];
            plot3(v(8),v(9),v(10),'bo');hold on;
            cntr=cntr+1;
            fprintf('.');
          end
        else
          fprintf('Ignore: %s\n',line);
        end
      end
      obj.writeline('x');
      obj.waitfor('+++END+++',5);
      obj.computecal();
    end
    
    function computecal(obj)
      mag=[obj.samples.m]';
      mtotal=sqrt(sum(mag.^2,2));
      fprintf('Mag total = %.0f+/-%.0f\n', mean(mtotal), std(mtotal));
      % Calibrate
      [A,b,expmfs]=magcal(mag)
      obj.newCal=struct('b',b,'A',A,'total',expmfs);
    end

    function calmag=applycal(obj,cal)
      % Calibrate data
      calmag=([obj.samples.m]'-cal.b)*cal.A;
      calmtotal=sqrt(sum(calmag.^2,2));
      fprintf('Cal Mag total = %.0f+/-%.0f\n', mean(calmtotal), std(calmtotal));
    end
    
    function showcal(obj)
      fprintf('{{%.2f,%.2f,%.2f},\n',obj.newCal.b);
      fprintf('{');
      for i=1:3
        if i>1
          fprintf(',\n');
        end
        fprintf('{%.2f,%.2f,%.2f}',obj.newCal.A(i,:));
      end
      fprintf('}}; // Total=%.2f\n',obj.newCal.total);
    end
    
    function plotmag(obj)
      setfig('mag');clf;
      tiledlayout('flow');
      nexttile;
      mag=[obj.samples.m]';
      maxval=max(max(abs(mag)));

      plot3(mag(:,1),mag(:,2),mag(:,3),'o');
      hold on;plot3([-maxval,maxval],[0,0],[0,0],'r');plot3([0,0],[-maxval,maxval],[0,0],'r');plot3([0,0],[0,0],[-maxval,maxval],'r');
      axis([-maxval maxval -maxval maxval -maxval maxval]);
      axis equal;
      xlabel('X');ylabel('Y');zlabel('Z');
      title('Raw');
      ax=gca;

      if ~isempty(obj.curCal)
        nexttile;
        fprintf('Current: ');
        calmag=obj.applycal(obj.curCal);
        plot3(calmag(:,1),calmag(:,2),calmag(:,3),'o');
        hold on;plot3([-maxval,maxval],[0,0],[0,0],'r');plot3([0,0],[-maxval,maxval],[0,0],'r');plot3([0,0],[0,0],[-maxval,maxval],'r');
        xlabel('X');ylabel('Y');zlabel('Z');
        title('Current Calibration');
        axis equal;
        ax(end+1)=gca;
      end

      if ~isempty(obj.newCal)
        nexttile;
        fprintf('New:     ');
        calmag=obj.applycal(obj.newCal);
        plot3(calmag(:,1),calmag(:,2),calmag(:,3),'o');
        hold on;plot3([-maxval,maxval],[0,0],[0,0],'r');plot3([0,0],[-maxval,maxval],[0,0],'r');plot3([0,0],[0,0],[-maxval,maxval],'r');
        xlabel('X');ylabel('Y');zlabel('Z');
        title('New Calibration');
        axis equal;
        ax(end+1)=gca;
      end

      Link = linkprop(ax,{'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
      setappdata(gcf, 'StoreTheLink', Link);
    end
    
    function savecal(obj)
      assert(~isempty(obj.newCal));
      buf=sprintf('IMAGSET %s',sprintf('%.3f,',obj.newCal.b,obj.newCal.A));
      buf=buf(1:end-1);
      obj.writeline(buf);
      obj.waitfor('MAGSET ok',1);
    end

    function plotacc(obj)
      setfig('acc');clf;
      tiledlayout('flow');
      nexttile;
      t=[obj.samples.t]/1000;
      t=t-t(1);
      acc=[obj.samples.a]'/2048;
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
    end
    
    function monitor(obj)
      while true
        data=obj.readline();
        fprintf('%s\n',data);
      end
    end
  end
  
end
