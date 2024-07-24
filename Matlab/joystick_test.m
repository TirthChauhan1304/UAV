joy = vrjoystick(1);


while 1
    
  b = zeros(10,1);
  
  for n = 1:10
      
  b(n) = button(joy,n);
  disp(b(n));
  
  end
  
  d = read(joy);
  disp(d);
  
  pause(0.02); 
  
end