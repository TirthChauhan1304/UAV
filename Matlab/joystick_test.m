joy = vrjoystick(1);


while 1
    
  b = zeros(12,1);
  
  for n = 1:12
      
  b(n) = button(joy,n);
  disp(b(n));
  
  end
  
  d = read(joy);
  disp(d);
  
  pause(0.25); 
  
end