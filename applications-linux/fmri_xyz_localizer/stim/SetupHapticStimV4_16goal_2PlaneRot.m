clear;
th = 0:pi/4:2*pi;
xmat = [];
for i=1:8,
  R= [cos(th(i)) -sin(th(i)); sin(th(i)) cos(th(i))];
  xmat(:,i) = R * [0.12;0]
end

xmat = [xmat;ones(1,8).*0];
th2 = pi/12; %acos(5/10)/2;

Rth2 = [cos(th2) 0 sin(th2); 
      0 1 0;
    -sin(th2) 0 cos(th2)];
  
x = Rth2*xmat;

th2 = -pi/10;

Rth2 = [cos(th2) 0 sin(th2); 
      0 1 0;
    -sin(th2) 0 cos(th2)];
  
x = [x, Rth2*xmat];

x = x(:,[1,2,4,5,6,8:end]);

for i=1:size(x,2),
  x(:,i) = x(:,i) + [0 0 0.05]';
end

figure(1); hold off; plot3(x(1,:),x(2,:),x(3,:),'o'); grid on; axis square;
xlim([-.2 .2]);ylim([-.2 .2]);zlim([-.2 .2]);



%%

repeat = 1;
while repeat == 1,
  % Create 15 stimuli each for rest(0) + corners(1-8)
  n_stims_per_run = 3; 
  idx = rem(randperm(28 * n_stims_per_run), 28);

  output = [];
  for i = 1:length(idx),
    if(idx(i) == 0)
      output = [output; 0 -1 3 0 0 0]; 
    else
      trest = 2+rem(floor(rand(1,1)*3),2);
      tplan = 1+rem(floor(rand(1,1)*3),2);
      tmove = 2+rem(floor(rand(1,1)*3),2);
      if(idx(i) < 14)
        output = [output;
          1 -1 trest 0 0 0;
          [[2 -1 tplan], x(:,idx(i)+1)'];
          [[3 -1 tmove], x(:,idx(i)+1)']];
      else
        tidx = idx(i) - 14;
        output = [output;
          1 -1 trest 0 0 0;
          [[2 -1 tplan], x(:,tidx+1)']];
      end
    end;
  end;

  tr = 1.57;
  n_trs = 254;
  t_end = ceil(n_trs * tr);
  output = [1 8 8 0 0 0; output; 1 t_end 5 0 0 0]; % 225 TRs

  for i = 2:size(output,1)-1,
    output(i,2) = output(i-1,2) + output(i,3);
  end

  output(end,3) = output(end,2) - output(end-1,2);
  
  if output(end,3) < 5 && output(end,3) > 2,
    repeat = 0;
  end
end


fp = fopen('delme.txt','w');
for i = 1:size(output,1),
  fprintf(fp, '%d %.3f %.3f %.3f %.3f %.3f\n', output(i,:));
end
fclose(fp);