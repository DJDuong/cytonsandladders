function IsCurtain = Arduino()
%Need to install add-on "MATLAB Support Package for Arduino Hardware"
a = arduino();
%a.AvailablePins
  while 1
  %readVoltage(a,'A0')
  volts = readVoltage(a,'A0')*0.0048828125;
  distance = 27.86*(volts.^ -1)/100
  pause(0.5)
  if distance <= 20
      IsCurtain = true;
  else
      IsCurtain = false;
  end
  end

end
