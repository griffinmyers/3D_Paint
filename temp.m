function [] = Paint()
% 3D Paint Program written by William Myers (wgm37) & Guo Jie Chin (gc348)
% for ECE 4760, Spring 2012. 
% Some serial communication code snippets adapted from Bruce Land
% http://people.ece.cornell.edu/land/PROJECTS/JordanMoore/BirdControl.m

% Close out and clean any open serial connections
clear all
close all
try
    fclose(instrfind); %close any bogus serial connections
end

% initiate a new serial communication object
s = serial('COM3','BaudRate',38500,'DataBits',8,'StopBits',1); 
fopen(s);

fprintf('%s', 'Executing...');
get(s,{'Name','Port','Type','BaudRate','Parity','StopBits','DataBits'})
s.Status
PushState = 1;
i = 0;
M = 10; % Median Filter Window Size
N = 10;  % Spline Window Size
median_window = zeros(M);  % Median Filter Vector
spline_window = zeros(N);
cycle_delay = 0;
k = 0;
    while(1)
        % request data from the microcontroller
        fprintf(s,'y');
        % wait for requested data from the microcontroller
        A = fscanf(s,'%d,%d');
        A = A';
        cycle_delay = A(2);
        point = cycle_delay / 16000000 * 340.29;
        %fprintf('%s\n\r', Switch);
        
        median_window = [point, median_window(1:length(median_window)-1)];
        filtered_point = median(median_window);
        spline_window = [spline_window(2:length(spline_window)), filtered_point];
        index = i - N + 1:i;
        t = i - N + 1:.1:i;
        y = pchip(index, spline_window,t);
        plot(y,t);
        axis([0 1 i-50 i+50])
        %plot(filtered_point,i,'r*',point,i,'go');
        
        i = i + 1;
        k = k + 1;
        hold on
        pause(.0001);
        if(i == 100)
           clf;
           i = 0;
        end
        
    end

end

