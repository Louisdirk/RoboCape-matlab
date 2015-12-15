function alpha = points2alpha(p1, p2, p3)

    chi1 = atan2(p2(2)-p1(2), p2(1)-p1(1));
    chi2 = atan2(p3(2)-p2(2), p3(1)-p2(1));
    
    psi2 = chi2 - chi1;
    
    alpha = (pi - psi2)/2;

end