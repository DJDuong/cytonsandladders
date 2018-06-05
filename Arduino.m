function IsCurtain = Arduino()
%Need to install add-on "MATLAB Support Package for Arduino Hardware"
%actual distance from IR to IR block is 32cm with the hole, robot base,
%hole config. However, there is error that causes the sensor to output
%between 30 and 34cm at non-intrusion. Recieving a value higher than 34cm
%means that something is within the 0 to 10 cm range according to
%https://a.pololu-files.com/picture/0J1124.1200.png?840efca805165798a42e14834a40b1e0
%An intrusion at the 30 to 32cm range may not be detected because of the
%error of the sensor.
  a = arduino();
  volts = readVoltage(a,'A0')*0.0048828125;
  distance = 27.86*(volts.^ -1)/100
  pause(0.01)
  if distance <= 30 || 34 <= distance
      IsCurtain = 1;
  else
      IsCurtain = 0;
  end
end
