if ~exist('s','var')
  serialportlist();
  devname='/dev/cu.usbmodem401';
  s=ardserial(devname);
end
d=s.getimutest(3000);
