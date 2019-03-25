function [ ] = plot_filtring( time,X,X_noisy,Xehat,Xphat )

msen = sum((X-X_noisy).^2);
msee = sum((X-Xehat).^2);
msep = sum((X-Xphat).^2);

MSE_nosiy=mean(msen);
MSE_EKF=mean(msee);
MSE_PF =mean(msep);


figure(1)

plot(X(1,:),X(2,:))
hold on
plot(X_noisy(1,:),X_noisy(2,:))
plot(Xehat(1,:),Xehat(2,:))
plot(Xphat(1,:),Xphat(2,:))

legend('Real Trajectory','Trajectory wo filtering','Trajectory of EKF','Trajectory of PF')
xlabel('X')
ylabel('Y')
grid on


figure(2)
semilogy(time,msen,time,msee,time,msep)
legend('MSE wo filtering','MSE of EKF','MSE of PF')
xlabel('Time')
ylabel('MSE')

end

