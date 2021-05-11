classdef ardserial < handle
  properties
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
      fprintf('Opening %s\n',devname);
      obj.port=serialport(devname,115200);
      if isempty(obj.port)
        error('Failed open of %s',devname);
      end
      fprintf('Opened %s\n', devname);
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
    
    function d=getimutest(obj,numsamples)
    % Put unit in IMU test mode and record samples
      obj.writeline('IMON');
      if (~obj.waitfor('+++IMON+++',1))
        d=[];
        return;
      end
      fprintf('In IMON mode\n');
      d=nan(numsamples,10);
      cntr=1;
      while cntr<=numsamples
        line=obj.readline();
        if isempty(line)
          fprintf('No data returned from readline()\n');
          return;
        end
        if strncmp(line,'Raw:',4)
          v=sscanf(line,'Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d');
          if length(v)~=10
            fprintf('Parse error: %s\n', line);
          else
            d(cntr,:)=v;
            cntr=cntr+1;
            fprintf('.');
          end
        else
          fprintf('Ignore: %s\n',line);
        end
      end
      obj.writeline('x');
      obj.waitfor('+++END+++',5);
    end
    
    function savecal(obj,b,A)
      buf=sprintf('IMAGSET %s',sprintf('%.3f,',b,A));
      buf=buf(1:end-1);
      obj.writeline(buf);
      obj.waitfor('MAGSET ok',1);
    end

    function monitor(obj)
      while true
        data=obj.readline();
        fprintf('%s\n',data);
      end
    end
  end
  
end
