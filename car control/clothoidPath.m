function u = clothoidPath(t)
   if t < 4
        u = [1;0];
   elseif t < 16
        u = [1; -0.1*floor((t - 2)/2)];
   else
        u = [0;0];
   end  
end