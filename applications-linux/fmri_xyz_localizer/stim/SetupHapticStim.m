lim = [0.3 0.15 0.1];
cube = [lim(1) lim(2) lim(3);
     lim(1) lim(2) -lim(3);
     lim(1) -lim(2) lim(3);
     lim(1) -lim(2) -lim(3);
     -lim(1) lim(2) lim(3);
     -lim(1) lim(2) -lim(3);
     -lim(1) -lim(2) lim(3);
     -lim(1) -lim(2) -lim(3)];
   
lim = [0.1 0.1 0.05];
cube = [lim(1) lim(2) lim(3);
     lim(1) -lim(2) lim(3);
     -lim(1) lim(2) lim(3);
     -lim(1) -lim(2) lim(3);
     lim(1) lim(2) lim(3);
     lim(1) -lim(2) lim(3);
     -lim(1) lim(2) lim(3);
     -lim(1) -lim(2) lim(3)];

% Create 15 stimuli each for rest(0) + corners(1-8)
idx = rem(randperm(9*10), 9);

output = [];
for i = 1:length(idx),
  if(idx(i) == 0)
    output = [output; 0 -1 3 0 0 0]; 
  else
    output = [output;
              1 -1 (1+rand(1,1)*4) 0 0 0;
              [[2 -1 2], cube(idx(i),:)];
              [[3 -1 4], cube(idx(i),:)]];
  end;
end;

output = [0 5 5 0 0 0; output; 0 770 5 0 0 0];
for i = 2:size(output,1)-1,
  output(i,2) = output(i-1,2) + output(i,3);
end

%%
fp = fopen('delme.txt','w');
for i = 1:size(output,1),
  fprintf(fp, '%d %.3f %.2f %.2f %.2f %.2f\n', output(i,:));
end
fclose(fp);