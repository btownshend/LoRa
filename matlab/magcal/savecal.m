if ~exist('s','var')
  s=ardserial();
end
s.savecal(cal.b,cal.A);

