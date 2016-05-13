%%  Plot all the sensors in 2D and 3D
clc; 
clear;

data = importdata('22_trajec1.csv');
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

figure(1)
clf;
hold all
title('Test - Biological - Swing net 1 - Stance net 1')
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');

axis([-0.1 0.6 -0.5 0 0 0.5])
grid on

for i = 0:2
    plot3( data2( :, i*3+3),data2( :, i*3+2), -data2( :, i*3+1) )
end
for i = 24:25
    plot3( data2( 1, i*3+3),data2( 1, i*3+2), -data2( 1, i*3+1), '*g' )
    plot3( data2( size(data2,1), i*3+3),data2( size(data2,1), i*3+2), -data2( size(data2,1), i*3+1), '*r' )
end
    %plot3( data2( :, 24*3+3),data2( :, 24*3+2), -data2( :, 24*3+1),'-r' )
    %plot3( data2( :, 25*3+3),data2( :, 25*3+2), -data2( :, 25*3+1),'-g' )
    %plot3( data2( :, 26*3+3),data2( :, 26*3+2), -data2( :, 26*3+1),'-b' ) 
   
%% Projection

body_point_start = [ data2( 1, 24*3+3) data2( 1, 24*3+2) ];
body_point_end = [ data2( size(data2,1), 24*3+3) data2( size(data2,1), 24*3+2)  ];

% Step 1: arrange points in a matrix format
points = [body_point_start; body_point_end];
% Step 2: find the mean of the points
avg = mean(points, 1);
% Step 3: subtract the mean from all points
subtracted = bsxfun(@minus, points, avg);
% Step 4 : perform SVD
[U W V] = svd(subtracted);
% Step 5: find the direction vector (which is the top singular value)
direction = V(:, 1);
% Optionally normalize the direction vector
direction = -direction ./ direction (1);

direction1 = [ direction(1) direction(2) 0 ]
rotation_matrix = vrrotvec2mat( vrrotvec( [ 0 0 1 ], direction1 ) )
direction1 = direction1*rotation_matrix
rotaton_matrix = transpose(rotation_matrix)

%plot3( data2( 1, i*3+3),data2( 1, i*3+2), -data2( 1, i*3+1), '*g' )


for i = 1:( size( data2,2 ) / 3)-1
    for j = 1:(size(data2,1) / 3)-1
        A = [ data2(j*3+2,i*3+3) data2(j*3+2,i*3+2) data2(j*3+2,i*3+1) ];
        A*rotation_matrix;
        data2(j*3+2,i*3+3)=A(1);
        data2(j*3+2,i*3+2)=A(2);
        data2(j*3+2,i*3+1)=A(3);
    end
end
    