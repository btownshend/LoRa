if ~exist('s','var')
  serialportlist();
  devname='/dev/cu.usbmodem401';
  s=ardserial(devname);
end
s.acquire(3000);
s.plotmag();
