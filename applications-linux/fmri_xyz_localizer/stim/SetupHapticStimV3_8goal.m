th = 0:pi/4:2*pi;
for i=1:8,
  R= [cos(th(i)) -sin(th(i)); sin(th(i)) cos(th(i))];
  x{i} = R * [0.12;0]
end

%%
lim = [0.3 0.15 0.1];

repeat = 1;
while repeat == 1,
  % Create 15 stimuli each for rest(0) + corners(1-8)
  n_stims_per_run = 5; 
  idx = rem(randperm(9 * n_stims_per_run), 9);

  output = [];
  for i = 1:length(idx),
    if(idx(i) == 0)
      output = [output; 0 -1 3 0 0 0]; 
    else
      trest = 2+rem(floor(rand(1,1)*3),2);
      tplan = 1+rem(floor(rand(1,1)*3),2);
      tmove = 2+rem(floor(rand(1,1)*3),2);
      output = [output;
                1 -1 trest 0 0 0;
                [[2 -1 tplan], x{idx(i)}',0.1];
                [[3 -1 tmove], x{idx(i)}',0.1]];
    end;
  end;

  tr = 1.57;
%   n_trs = 225;
  n_trs = 171;
  t_end = ceil(n_trs * tr);
  output = [0 8 8 0 0 0; output; 0 t_end 5 0 0 0]; % 225 TRs

  for i = 2:size(output,1)-1,
    output(i,2) = output(i-1,2) + output(i,3);
  end

  output(end,3) = output(end,2) - output(end-1,2);
  
  if output(end,3) < 5 && output(end,3) > 2,
%   if output(end,3) > 2,
    repeat = 0;
  end
end


fp = fopen('delme.txt','w');
for i = 1:size(output,1),
  fprintf(fp, '%d %.3f %.2f %.2f %.2f %.2f\n', output(i,:));
end
fclose(fp);