function Zi = LMmeas(X, i)


x=X(1,1);
y=X(2,1);
phi=X(3,1);

xi=X(3+2*i-1,1);
yi=X(3+2*i,1);

Zi=[sqrt( (xi-x)^2 + (yi-y)^2 ) ;  atan2(  (yi-y),(xi-x)  )-phi];

end

