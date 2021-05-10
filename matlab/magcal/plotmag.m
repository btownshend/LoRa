mag=d(:,8:10);
mtotal=sqrt(sum(mag.^2,2));
fprintf('Mag total = %.0f+/-%.0f\n', mean(mtotal), std(mtotal));

setfig('mag');clf;
plot3(mag(:,1),mag(:,2),mag(:,3),'o');
axis equal
xlabel('X');
ylabel('Y');
zlabel('Z');

% Calibrate
[A,b,expmfs]=magcal(mag)

% Calibrate data
calmag=(mag-b)*A;

calmtotal=sqrt(sum(calmag.^2,2));
fprintf('Cal Mag total = %.0f+/-%.0f\n', mean(calmtotal), std(calmtotal));

hold on;
plot3(calmag(:,1),calmag(:,2),calmag(:,3),'o');
