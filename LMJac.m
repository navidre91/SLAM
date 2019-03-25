function Hi = LMJac(X, i)


x=X(1,1);
y=X(2,1);
phi=X(3,1);

xi=X(3+2*i-1,1);
yi=X(3+2*i,1);


Hi=[ -(xi-x)/sqrt( (xi-x)^2 + (yi-y)^2 ), -(yi-y)/sqrt( (xi-x)^2 + (yi-y)^2 ), 0;
    (yi-y)/(xi-x)^2/(  (yi-y)^2/(xi-x)^2  +1 ), -1/(xi-x)/(  (yi-y)^2/(xi-x)^2  +1 ), -1 ];


% Hi_rob=[ -(xi-x)/sqrt( (xi-x)^2 + (yi-y)^2 ), -(yi-y)/sqrt( (yi-y)^2 + (yi-y)^2 ), 0;
%     (yi-y)/(xi-x)^2/(  (yi-y)^2/(xi-x)^2  +1 ), -1/(xi-x)/(  (yi-y)^2/(xi-x)^2  +1 ), -1 ];
% 
% 
% Hi_zi=[ (xi-x)/sqrt( (xi-x)^2 + (yi-y)^2 ), (yi-y)/sqrt( (yi-y)^2 + (yi-y)^2 );
%     -(yi-y)/(xi-x)^2/(  (yi-y)^2/(xi-x)^2  +1 ), 1/(xi-x)/(  (yi-y)^2/(xi-x)^2  +1 )];
% 
% 
% Hi=zeros(2,9);
% Hi(:,1:3)=Hi_rob;
% Hi(:,3+2*i-1:3+2*i)=Hi_zi;


end

