%%  Import and ready the data.
clc; 
clear;

data = importdata('22_5.csv');
data1 = data;

data1( 1,: ) = [];
tmp = data1;
tmp( :, ~any(tmp,1) ) = [];  %columns
tmp1 = tmp;
tmp1( ~any(tmp1,2), : ) = [];  %rows
tmp2 = tmp1;
tmp2( ~all(tmp2,2), : ) = [];  %rows
data2 = tmp2;

for i = 0:20
    data2(1,:)=[];
end

%%  Make the rotation matrix for projection.
%   The goal is to project the data down to the x-axis, so that the start
%   and the end is on the x-axis. This will make it better for presentation
%   and to compare the different data sets.
clc;
%   Find the body part that everything will be oriented at.
Part = 24;
start_point = [ data2( 1, Part*3+3) data2( 1, Part*3+2) ];
end_point = [ data2( size(data2,1), Part*3+3) data2( size(data2,1), Part*3+2) ];
%new_end_point = [ end_point-start_point 0 ]
new_end_point = [end_point-start_point];

%   Here we find the angle that we need to rotate 
%   the points, to put it on the x-axis.
r = -acos( dot(new_end_point, [1 0])/( norm(new_end_point)*norm( [1 0] ) ) );

%rotation_matrix = [ cos(r) -sin(r) 0; sin(r) cos(r) 0; 0 0 1 ];
rotation_matrix = [ 1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r) ];

%%  Rotate the data.
for i = 0:26
    data3(:,i*3+1:i*3+3) = [ data2(:,i*3+1) data2(:,i*3+2) data2(:,i*3+3) ]*rotation_matrix;
end

figure(1)
hold on
grid on
axis([-1 25 -13 13 -5 5])
for i = 0:26
    plot3( data2( :, i*3+3),data2( :, i*3+2),-data2( :, i*3+1) )
end
%close(figure(1))

figure(2)
hold on
grid on
axis([-1 25 -13 13 -5 5])
for i = 0:26
    plot3( data3( :, i*3+3),data3( :, i*3+2),-data3( :, i*3+1) )
end
%close(figure(2))

%%  Run the statistic analysis on the data.
%%  Analysis the data of the height
%   We are interested in knowning what the mean, variance and the standard
%   deviation of the data is.

%   Making a vector that holds the height value for the three body parts.
A = [ -data3( :, 24*3+1) -data3( :, 25*3+1) -data3( :, 26*3+1) ];
%   It is now possible to find the mean, and variance of the height.
meanA = mean(A)
varA = var(A)
stdA = std(A)

%%  Analysis the data of the walking on the x-axis.
%   We are interested in knowning what the mean, variance and the standard
%   deviation of the data is.

%   Making a vector that holds the data value for the three body parts on
%   the x-axis
B = [ -data3( :, 24*3+2) -data3( :, 25*3+2) -data3( :, 26*3+2) ];
%   It is now possible to find the mean, and variance of the height.
meanB = mean(B)
varB = var(B)
stdB = std(B)

%%
C = [ -data3( :, 24*3+3) -data3( :, 25*3+3) -data3( :, 26*3+3) ];
%   It is now possible to find the mean, and variance of the height.
meanC = mean(C)
varC = var(C)
stdC = std(C)



