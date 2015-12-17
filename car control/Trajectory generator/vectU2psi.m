function psi = vectU2psi(dir1,dir2)

psi = acos(dot(dir2,dir1))*sign(det([dir1,dir2]));

end