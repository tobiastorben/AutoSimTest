%© Norwegian University of Science and Technology (NTNU),
%Department of Marine Technology.
%The software is developed under the ORCAS Project.
%Author: Tobias Rye Torben.
function [cpa, tcpa] = cpa_calculations(p1, Xi1, U1, p2, Xi2, U2)

n = length(Xi1);
cpa = zeros(1,n);
tcpa = zeros(1,n);
for i = 1:n
    a = p2(1,i)-p1(1,i);
    b = U2(i)*cos(Xi2(i))-U1(i)*cos(Xi1(i));
    c = p2(2,i)-p1(2,i);
    d = U2(i)*sin(Xi2(i))-U1(i)*sin(Xi1(i));
    
    tcpa(i) = -(a*b + c*d) / (b^2 + d^2);
    if tcpa(i) < 0
        cpa(i) = 1e12;
        tcpa(i) = 1e12;
    else
        cpa(i) = sqrt((a + b*tcpa(i))^2 + (c + d*tcpa(i))^2);
    end
end


end