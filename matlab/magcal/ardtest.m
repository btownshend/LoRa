if ~exist('s','var')
  serialportlist();
  devname='/dev/cu.usbmodem401';
  s=ardserial(devname);
end
s.writeline('x');   % To cancel any prior
pause(1);
s.flush();
s.writeline('LAT+VER');
if (~s.waitfor('<LORA: +VER',1))
  return;
end
