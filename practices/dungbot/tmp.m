figure(2)
clf;
hold all
title('Right and body')
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');
axis([-3.5 3.5 -3.5 3.5 0 7])
grid on

for i = 12:23
    plot3( data2( :, i*3+1),data2( :, i*3+2), data2( :, i*3+3) )
end
for i = 24:26
    plot3( data2( 1, i*3+1),data2( 1, i*3+2), data2( 1, i*3+3), '*g' )
    plot3( data2( size(data2,1), i*3+1),data2( size(data2,1), i*3+2), data2( size(data2,1), i*3+3), '*r' )
end
    plot3( data2( :, 24*3+1),data2( :, 24*3+2), data2( :, 24*3+3),'--r' )
    plot3( data2( :, 25*3+1),data2( :, 25*3+2), data2( :, 25*3+3),'--g' )
    plot3( data2( :, 26*3+1),data2( :, 26*3+2), data2( :, 26*3+3),'--b' )





