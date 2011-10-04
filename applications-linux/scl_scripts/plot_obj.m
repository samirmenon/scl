vert = load('/home/samir/Code/sai/scl/busylizzy.git/trunk/specs/Puma/graphics/base2.obj')
line = load('/home/samir/Code/sai/scl/busylizzy.git/trunk/specs/Puma/graphics/base2b.obj');

figure(1); hold off;
plot3(vert(:,1),vert(:,2),vert(:,3),'r*'); hold on; 

for i = 1:size(line,1),
  plot3(vert(line(i,[1 3 5]),1),vert(line(i,[1 3 5])',2),vert(line(i,[1 3 5]),3),'b');
end

grid on; axis on;