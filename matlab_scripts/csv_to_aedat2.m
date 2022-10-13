clearvars
close all;
dbstop if error 

aedat = struct;

%-------------------------------- USER INPUT --------------------------------% 
csv_path = '/path/to/input/csv/file';
aedat.exportParams.filePath = '/path/to/output/aedat2/file';
aedat.info.deviceAddressSpace = [128,128];
aedat.info.fileFormat = 2;
aedat.info.source = 'Dvs128';
%----------------------------------------------------------------------------%

csv_table = readtable(csv_path);

polar = table2array(csv_table(:,4));

aedat.data.polarity.polarity = logical(polar);
aedat.data.polarity.timeStamp = uint32(1000000 * table2array(csv_table(:,3)));
aedat.data.polarity.y = uint16(table2array(csv_table(:,2)));
aedat.data.polarity.x = uint16(table2array(csv_table(:,1)));
aedat.data.polarity.numEvents  = length(polar);

ExportAedat2Polarity(aedat)