clear, clc

data = csvread("data.csv", 1, 0);

inverse_distance = 1./data(:,2);

p = polyfit(data(:,1),inverse_distance,1);

fileID = fopen('fit.txt','w');
fprintf(fileID,'%6s\n','AnalogRead [0-4096] | Distance [cm]');
fprintf(fileID,'Distance = 1/(%.15f*AnalogRead + %.15f)\n',p(1),p(2));
fclose(fileID);