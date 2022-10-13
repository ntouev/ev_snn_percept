clearvars
close all;
dbstop if error
 
aedat = struct; 

%-------------------------------- USER INPUT --------------------------------% 
aedat.source = 'Dvs128';  
% input .aedat file
aedat.importParams.filePath = '~/ev_snn_percept/aedat2/sample_pavement.aedat'; 
% directory to store the the generated CSVs
store_dir = '~/ev_snn_percept/binary/sample_pavement/csv/';
% define the time instant (in seconds) where the event accumulation into frames starts
aedat.importParams.startTime = 0;
% define the time instant (in seconds) where the event accumulation into frames stops
aedat.importParams.endTime = 16;
% the milliseconds of the simulation, i.e. (endTime - startTime)*1000
simtime = 16000;
% accumulation dt in msec for each frame
accumulation_dt = 50;
% make sure this division gives an int
nbr_of_plots = simtime/accumulation_dt; 
%----------------------------------------------------------------------------%

aedat = ImportAedat(aedat);

% monotonicity check
temp = aedat.data.polarity.timeStamp(1);
aedat.data.polarity.timeStamp(1) = 0;
for i = 2:aedat.data.polarity.numEvents
    if aedat.data.polarity.timeStamp(i) < aedat.data.polarity.timeStamp(i-1)
        error('Monotonicicy error!')
    end
end
aedat.data.polarity.timeStamp(1) = temp; % restore first timestamp

bin_image = zeros(128, 128, nbr_of_plots);
for i = 1:aedat.data.polarity.numEvents
    t = fix(fix(aedat.data.polarity.timeStamp(i)/1000)/fix(simtime/nbr_of_plots)) + 1;
    if (aedat.data.polarity.polarity(i) == 1) 
        bin_image(aedat.data.polarity.y(i)+1, aedat.data.polarity.x(i)+1, t) = 1;
    end
    if (aedat.data.polarity.polarity(i) == 0)
        bin_image(aedat.data.polarity.y(i)+1, aedat.data.polarity.x(i)+1, t) = -1;
    end
end

for i = 1:nbr_of_plots
    str = strcat(store_dir, num2str(sprintf('%05d',i-1))); % i-1 to store in zero-based format
    str = strcat(str,'.csv');
    writematrix(squeeze(bin_image(:, :, i)), str);
end