lim = [0.3 0.15 0.1];
cube = [lim(1) lim(2) lim(3);
     lim(1) lim(2) -lim(3);
     lim(1) -lim(2) lim(3);
     lim(1) -lim(2) -lim(3);
     -lim(1) lim(2) lim(3);
     -lim(1) lim(2) -lim(3);
     -lim(1) -lim(2) lim(3);
     -lim(1) -lim(2) -lim(3)];
   
lim = [0.1 0.1 0.05]; % Repeated. So n-trials = x2
cube = [lim(1) lim(2) lim(3);
     lim(1) -lim(2) lim(3);
     -lim(1) lim(2) lim(3);
     -lim(1) -lim(2) lim(3);
     lim(1) lim(2) lim(3);
     lim(1) -lim(2) lim(3);
     -lim(1) lim(2) lim(3);
     -lim(1) -lim(2) lim(3)];

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
      trest = 2+rem(floor(rand(1,1)*3),3);
      tplan = 1+rem(floor(rand(1,1)*3),3);
      tmove = 3;
      output = [output;
                1 -1 trest 0 0 0;
                [[2 -1 tplan], cube(idx(i),:)];
                [[3 -1 tmove], cube(idx(i),:)]];
    end;
  end;

  tr = 1.57;
  n_trs = 225;
  t_end = ceil(n_trs * tr);
  output = [0 8 8 0 0 0; output; 0 t_end 5 0 0 0]; % 225 TRs

  for i = 2:size(output,1)-1,
    output(i,2) = output(i-1,2) + output(i,3);
  end

  output(end,3) = output(end,2) - output(end-1,2);
  
  if output(end,3) < 5 && output(end,3) > 2,
    repeat = 0;
  end
end

%%
fp = fopen('delme.txt','w');
for i = 1:size(output,1),
  fprintf(fp, '%d %.3f %.2f %.2f %.2f %.2f\n', output(i,:));
end
fclose(fp);