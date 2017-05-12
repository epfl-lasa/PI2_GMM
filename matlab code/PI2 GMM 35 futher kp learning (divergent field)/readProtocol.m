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
  d = textscan(line,'%f %f %f %f %f %d %s %d %d %d %d %d %d %d %d %s %d %d %d %f %s %f %d');
  count = count+1;
  % the <protocol> structure stores all important parameters to run PI2
  % scenario 
  protocol(count).start       = [d{1};d{2}];
  protocol(count).duration    = d{3};
  protocol(count).dt          = d{4};
  protocol(count).std         = d{5};
  protocol(count).rep         = d{6};
  protocol(count).cost        = char(d{7});
  protocol(count).updates     = d{8};
  protocol(count).fixed_noise = d{9};
  protocol(count).selective_noise = d{10};
  protocol(count).n_reuse     = d{11};
  protocol(count).gaussian_centers = d{12};
  protocol(count).feedback    = d{13};
  protocol(count).SEDS_constr    = d{14};
  protocol(count).SEDS_init     = d{15};
  protocol(count).PI2_type      = d{16};
  protocol(count).n_runs        = d{17};
  protocol(count).relearnGMM    = d{18};
  protocol(count).n_Gauss       = d{19};
  protocol(count).kp0       = d{20};
  protocol(count).demo_set  = d{21};
  protocol(count).duration_convergence  = d{22};
  protocol(count).disable_plotting = d{23};
end
fclose(fp);
end