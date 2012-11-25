xyzpos = [0.15  0.15  0.08;
          0.15 -0.15  0.08;
          0.15  0.15  0.03;
          0.15 -0.15  0.03;
             0  0.15  0.08;
             0 -0.15  0.08;
             0  0.15  0.03;
             0 -0.15  0.03;
         -0.15  0.15  0.08;
         -0.15 -0.15  0.08;
         -0.15  0.15  0.03;
         -0.15 -0.15  0.03;];

xyzforce = [1  0  0;
            0  1  0;
            0  0  1;
           -1  0  0;
            0 -1  0;
            0  0 -1];

%%

repeat = 1;
while repeat == 1,
  % Create 15 stimuli each for rest(0) + corners(1-8)
  n_stims_per_run = 1; 
  % Add +1 at the end to remove all 0 rests.
  n_rest_states = 2;
  idx = rem(randperm( (size(xyzpos,1)+n_rest_states) * n_stims_per_run), size(xyzpos,1)+n_rest_states ) - n_rest_states + 1;
  
  output = [];
  for i = 1:length(idx),
    if(idx(i) <= 0)
      output = [output; 0 -1 5 0 0 0 0 0 0]; %5s random rest
    else
      fidx = randperm(size(xyzforce,1));
      % 2-4s rest + 3s reach + 6 * ( 2-4s hold + 2s F)
      trest = 2+rem(floor(rand(1,1)*3),2);
      tmove = 3;
      output = [output;
                [3 -1 tmove], xyzpos(idx(i),:), 0, 0, 0];
              for j = 1:length(fidx),
                thold = 2+rem(floor(rand(1,1)*3),2);
                output = [  output;
                            [3 -1 thold], xyzpos(idx(i),:), 0, 0, 0;
                            [3 -1 thold], xyzpos(idx(i),:), xyzforce(fidx(j),:);
                         ];
              end
              
      output = [output;
                [3 -1 trest], xyzpos(idx(i),:), 0, 0, 0];
    end;
  end;

  tr = 1.57;
  n_trs = 280;
  t_end = ceil(n_trs * tr);
  output = [0 8 8 0 0 0 0 0 0; output; 0 t_end 5 0 0 0 0 0 0]; % 225 TRs

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
  fprintf(fp, '%d %.3f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n', output(i,:));
end
fclose(fp);

output