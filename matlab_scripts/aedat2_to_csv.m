clearvars
close all;
dbstop if error

aedat = struct;

%-------------------------------- USER INPUT --------------------------------% 
aedat.source = 'Dvs128'; 
aedat.importParams.filePath = '~/ev_snn_percept/aedat2/sample_pavement.aedat';
csv_path = '~/ev_snn_percept/CSVs/sample_pavement.csv';
% % 150 kHz limiting - adjust accordingly only in case of fpga simulation - event rate limiting
T = 1/150000 * 1000000; % usec (FOR EPS LIMMITING!)
%----------------------------------------------------------------------------%

aedat = ImportAedat(aedat);

%------------------------- NO EVENT RATE LIMITING ---------------------------%
M = zeros(aedat.data.polarity.numEvents, 4);

M(:, 1) = aedat.data.polarity.x;
M(:, 2) = aedat.data.polarity.y;
M(:, 3) = double((aedat.data.polarity.timeStamp) - aedat.data.polarity.timeStamp(1))/1000000;
M(:, 4) = aedat.data.polarity.polarity;

writematrix(M, csv_path);

% %------------------ FPGA SIMULATION - EVENT RATE LIMMITING -----------------%
% M_150 = zeros(aedat.data.polarity.numEvents, 4);
% prev_timestamp = aedat.data.polarity.timeStamp(1);
% j = 1;
% for i = 1:aedat.data.polarity.numEvents
%    current_timestamp = aedat.data.polarity.timeStamp(i);
%    if current_timestamp - prev_timestamp > T
%        M_150(j, 1) = aedat.data.polarity.x(i);
%        M_150(j, 2) = aedat.data.polarity.y(i);
%        M_150(j, 3) = double(aedat.data.polarity.timeStamp(i) - aedat.data.polarity.timeStamp(1))/1000000;
%        M_150(j, 4) = aedat.data.polarity.polarity(i);
      
%        j = j + 1;
%        prev_timestamp = current_timestamp;
%    end
% end
% M_150( all(~M_150,2), : ) = []; % remove the remaining rows of zeros

% writematrix(M_150, csv_path);