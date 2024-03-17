clc
clear all

load TRACKING.txt
main =TRACKING(1:end,:);
main(:,1) = main(:,1) ;
[g,x] =  size(main);
S =  main(:,4);
horizontal_angle = main(:,2)*pi/200 ;
verticle_angle = main(:,3)*pi/200 ;
t = main(:,5);
index_v=200;  
total_number=100;
num_points_obtain=150;

wrong_X = S.*cos(horizontal_angle).*sin(verticle_angle);
wrong_Y = S.*sin(horizontal_angle).*sin(verticle_angle);
wrong_Z = S.*cos(verticle_angle);
wrong_position = [wrong_X,wrong_Y,wrong_Z];
%scatter(wrong_X,wrong_Y);

E = zeros(g,1);
for i = 2:g
   E(i) = sqrt((wrong_X(i)-wrong_X(i-1))^2+(wrong_Y(i)-wrong_Y(i-1))^2+(wrong_Z(i)-wrong_Z(i-1))^2); 
end

time_parameter = zeros(g,1);
verticle_angle_parameter = zeros(g,1);
V = zeros(g,1);

for i = 2:g-1
    

    time_parameter(i) = atan((wrong_Y(i+1)-wrong_Y(i-1))/(wrong_X(i+1)-wrong_X(i-1)));
    verticle_angle_parameter(i) = acos((wrong_Z(i+1)-wrong_Z(i-1))/(E(i+1)+E(i)));
    V(i) = (E(i+1)+E(i))/(t(i+1)-t(i-1));
end

cos_a = cos(verticle_angle_parameter).*cos(verticle_angle_parameter) + sin(verticle_angle_parameter).*sin(verticle_angle_parameter).*cos(time_parameter-horizontal_angle);

figure(1);
plot(1:size(V,1),V);
title('Velocity  Diagram');
xlabel('Number of points Detected');
ylabel('Values is in [m/s]');

% Finding the with the help of equation the latency time
line = sortrows(main,2);
group8 = [];

i = 1;
while i<size(line,1)
    if abs(line(i,1)-line(i+1,1))~=1        % && abs(line(i,1)-line(i+1,1))<0.9
        group8 = [group8;line(i,:);line(i+1,:)];
        i = i+1;
    end
    i = i+1;
end

num_point = size(group8,1)/2;
f = zeros(num_point,5);
r = zeros(num_point,5);
 
for i = 1:num_point
    m = group8(2*i-1,1);
    n = group8(2*(i),1);
    if m<n
        f(i,:) = group8(2*(i-1),:);
        r(i,:) = group8(2*i,:);
    else
        r(i,:) = group8(2*i-1,:);
        f(i,:) = group8(2*i,:);
    end  
end

f = f(1:101,:);
r = r(1:101,:);

num_point = size(f,1);

index_f = f(:,1);
index_r = r(:,1);

t_latency = ( wrong_Y(index_r) - wrong_Y(index_f) ) ./ ( 2 .* V(index_f) .* cos_a(index_f) .* sin(horizontal_angle(index_f)) .* sin(verticle_angle(index_f)).*index_v);

for i = 1:1:76
    if abs(t_latency(i)>0.9) 
   
         t_latency(i,:) =[];
        f(i,:) = [];
        r(i,:) = [];
        index_f(i,:) = [];
        index_r(i,:) = [];
  
    end
end

mean_velocity= mean(V);
mean_t_latency=mean(t_latency);

figure(2);
plot(1:size(t_latency,1),t_latency);
title('Latency Time');
xlabel('Number of points');
ylabel('Latency [s]');

delta_s_f = V(index_f)*mean(t_latency);
delta_s_r = V(index_r)*mean(t_latency);

true_Y_f = wrong_Y(index_f)+delta_s_f.*(cos_a(index_f)-0.5*delta_s_f./S(index_f).*(1-cos_a(index_f).^2)).*sin(horizontal_angle(index_f)).*sin(verticle_angle(index_f));
true_X_f = wrong_X(index_f)+delta_s_f.*(cos_a(index_f)-0.5*delta_s_f./S(index_f).*(1-cos_a(index_f).^2)).*cos(horizontal_angle(index_f)).*sin(verticle_angle(index_f));
true_Z_f = wrong_Z(index_f)+delta_s_f.*(cos_a(index_f)-0.5*delta_s_f./S(index_f).*(1-cos_a(index_f).^2)).*cos(verticle_angle(index_f));

true_Y_r = wrong_Y(index_r)+delta_s_r.*(cos_a(index_r)-0.5*delta_s_r./S(index_r).*(1-cos_a(index_r).^2)).*sin(horizontal_angle(index_r)).*sin(verticle_angle(index_r));
true_X_r = wrong_X(index_r)+delta_s_r.*(cos_a(index_r)-0.5*delta_s_r./S(index_r).*(1-cos_a(index_r).^2)).*cos(horizontal_angle(index_r)).*sin(verticle_angle(index_r));
true_Z_r = wrong_Z(index_r)+delta_s_r.*(cos_a(index_r)-0.5*delta_s_r./S(index_r).*(1-cos_a(index_r).^2)).*cos(verticle_angle(index_r));

figure(3);
plot(wrong_X(index_f),wrong_Y(index_f),'r*');
hold on
plot(wrong_X(index_r),wrong_Y(index_r),'g*');
title('wrong Positon');
xlabel('X [meter]');
ylabel('Y [meter]');
legend('FRONT','BACK');

figure(4);
plot(true_X_f,true_Y_f,'r+');

hold on
plot(true_X_r,true_Y_r,'g+');
title('Real location');
xlabel('X [meter]');
ylabel('Y [meter]');
legend('FRONT','BACK');




