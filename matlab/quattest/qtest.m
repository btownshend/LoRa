% Test out angles for tilting device
% World frame is NEU (x-North, y-East, z-Up)
% Given a set of Euler angles (ZYX), figure out which the N direction is pointing (in world x,y,z)
% Should be identical to the accelerometer current reading
econvert(0,-90,0);
econvert(90,-90,0);
econvert(0,-85,85);
econvert(0,-60,90);
econvert(0,-45,45);
econvert(0,-10,10);
econvert(45,-90,90);
econvert(0,0,-90);
econvert(90,0,-90);


function pt=econvert(yaw,pitch,roll)
  q=quaternion([yaw,pitch,roll],'eulerd','ZYX','frame');
  fprintf('Q=[%.2f,%.2f,%.2f,%.2f]\n', compact(q));
  px=rotatepoint(q,[1,0,0]);
  py=rotatepoint(q,[0,1,0]);
  pz=rotatepoint(q,[0,0,1]);
  clock=atan2(py(3),px(3))*180/pi;
  tilt=acos(pz(3))*180/pi;
  fprintf('Y=%3.0f, P=%3.0f, R=%3.0f: [1,0,0] -> [%5.2f,%5.2f,%5.2f], [0,1,0] -> [%5.2f,%5.2f,%5.2f] ', yaw,pitch,roll,px,py);
  x2=-sind(pitch);
  y2=cosd(pitch)*sind(roll);
  z2=cosd(pitch)*cosd(roll);
  fprintf('Clock=%5.0f, Tilt=%5.0f, ',clock, tilt);
  fprintf('[x,y,z]=[%5.2f,%5.2f,%5.2f], ',x2,y2,z2);
  ea=eulerd(q,'YXZ','frame');
  fprintf('P=%3.0f, R=%3.0f, Y=%3.0f\n\n', ea);
  keyboard;
end


