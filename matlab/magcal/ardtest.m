if ~exist('s','var')
  serialportlist();
  devname='/dev/cu.usbmodem14601';
  s=ardserial(devname);
end
s.writeline('x');   % To cancel any prior
s.flush();
s.writeline('LAT+VER');
if (~s.waitfor('<LORA: +VER',1))
  return;
end
