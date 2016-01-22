a = '$PUBX,41,1,0007,0003,203400,0';
cksum = 0;

for i = 2:length(a)
    cksum = bitxor(cksum,double(a(i)));
end

disp([a '*' num2str(dec2hex(cksum))]);