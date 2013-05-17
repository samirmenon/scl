close all;
clear;

%% Create a force-only file.
forces = {[1 0 0], [1 0 0], [0 1 0], [-1 0 0], [0 -1 0], [0.707 0.707 0], ...
  [0.707 -0.707 0], [-0.707 0.707 0],[-0.707 -0.707 0]};
bkg = {1,0,0,0,0,0,0,0,0};

% * 0          1                    2                    3       4       5       6        7        8
% * <task-id> <time-at-completion> <background-enabled> <x-des> <y-des> <z-des> <Fx-des> <Fy-des> <Fz-des>
% All functional responses (visual stim or forces) run for 3TRs.
       
noptions = size(forces,2);

repeat = 1;
tr = 1.57;
n_trs = 275;
n_stims_per_run = 4;
n_total_runs = 7;
max_force = 1.2;
z_disp = 0.07;

for irun = 0:1:n_total_runs,
  repeat =1;
  while repeat == 1,
    % Create 15 stimuli each for rest(0) + corners(1-8)
    tcurr = 5*tr; % waste 5 trs at the start
    output = [0 tcurr 0 0 0 0 0 0 0];

    idx = rem(randperm((noptions+1) * n_stims_per_run), (noptions+1));
    idx = [idx, zeros(1,n_stims_per_run*3)];
    idx = idx(randperm(length(idx)));

    for i = 1:length(idx),
      if(idx(i) == 0)
        tcurr = tcurr+2*tr; %2 trs of rest + no visual flashing
        output = [output; idx(i) tcurr 0 0 0 0 0 0 0]; 
        
        tcurr = tcurr+3*tr; %3 trs of rest + visual flashing
        % Special id = 99.
        if rand(1,1) > 0.5,
          output = [output; 99 tcurr 1 0 0 0 0 0 0];
        else
          output = [output; idx(i) tcurr 0 0 0 0 0 0 0]; 
        end
        
        tcurr = tcurr+tr; %1 tr of rest + no visual flashing
        output = [output; idx(i) tcurr 0 0 0 0 0 0 0]; 
      else
        tmove = tr+round(rand(1,1)*2)*tr; % 1-3 trs of motion time
        tforce = 3*tr;
        % No flashing during move. Only during force application
        output = [output;
          (idx(i)+noptions) (tcurr+tmove) 0 0 0 z_disp 0 0 0;
          idx(i) (tcurr+tmove+tforce) bkg{idx(i)} 0 0 z_disp max_force*forces{idx(i)}];
        tcurr = tcurr+tmove+tforce;
      end;
    end;

    t_end = ceil(n_trs * tr) - tcurr;
    if t_end>3 && t_end < 5,
      tcurr = tcurr+t_end;
      output = [output; 0 tcurr 0 0 0 0 0 0 0]; % Pad the end with 3-5s of zeros
      repeat = 0;
    end
  end


  % Set outfile
  outfile = sprintf('./testVision/%s_forces%d.txt', date,irun);

  % Write to the file.
  fp = fopen(outfile, 'w');
  for i=1:size(output,1)-1,
    fprintf(fp, '%d %8.4f %d %6.3f %6.3f %6.3f %3.2f %3.2f %3.2f\n',...
      output(i,1), output(i,2), output(i,3), output(i,4), output(i,5), output(i,6), output(i,7), output(i,8), output(i,9));
  end
  i = i+1;
  fprintf(fp, '%d %8.4f %d %6.3f %6.3f %6.3f %3.2f %3.2f %3.2f',...
      output(i,1), output(i,2), output(i,3), output(i,4), output(i,5), output(i,6), output(i,7), output(i,8), output(i,9));
  fclose(fp);
end