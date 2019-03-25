function MSE = calc_MSE( X,X_rec )

xy=X(1:2,:);
xy_rec=X_rec(1:2,:);

xy_diff=xy-xy_rec;

xy_diff_p2=xy_diff.^2;

xy_diff_p2_sum=sum(xy_diff_p2,1);

MSE=sum(sqrt(xy_diff_p2_sum))/size(X,2);

end

