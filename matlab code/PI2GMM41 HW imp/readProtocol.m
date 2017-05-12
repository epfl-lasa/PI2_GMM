function protocol=readProtocol(protocol_name)
% parses the protocol file <protocol_name> 

fp = fopen(protocol_name,'r');
if fp == -1,
  error('Cannot open protocol file');
end

% read all lines, discard lines that start with comment sign
protocol = [];
count    = 0;
while 1,
  line = fgetl(fp);
  if ~ischar(line),
    break;
  end
  if numel(line) == 0 || line(1) == '%',
    continue;
  end
  d = textscan(line,'%f %f %f %f %f %f %d %s %d %d %d %d %d %d %d %d %s %d %d %d %f %s %f %d %d %f %f %f %f %f %f');
  count = count+1;
  % the <protocol> structure stores all important parameters to run PI2
  % scenario 
  protocol(count).start       = [d{1};d{2};d{3}];
  protocol(count).duration    = d{4};
  protocol(count).dt          = d{5};
  protocol(count).std         = d{6};
  protocol(count).rep         = d{7};
  protocol(count).cost        = char(d{8});
  protocol(count).updates     = d{9};
  protocol(count).fixed_noise = d{10};
  protocol(count).selective_noise = d{11};
  protocol(count).n_reuse     = d{12};
  protocol(count).gaussian_centers = d{13};
  protocol(count).feedback    = d{14};
  protocol(count).SEDS_constr    = d{15};
  protocol(count).SEDS_init     = d{16};
  protocol(count).PI2_type      = d{17};
  protocol(count).n_runs        = d{18};
  protocol(count).relearnGMM    = d{19};
  protocol(count).n_Gauss       = d{20};
  protocol(count).kp0       = d{21};
  protocol(count).demo_set  = d{22};
  protocol(count).duration_convergence  = d{23};
  protocol(count).disable_plotting = d{24};
  protocol(count).HWinLoop = d{25};
  protocol(count).offset = [d{26};d{27};d{28};d{29};d{30};d{31}];
end
fclose(fp);
end