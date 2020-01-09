% Author: Justin Ngo
% 7/1/2019
% University of Utah: DARC Lab
    
% Client for compression device to test ionic polymer metal composites
% This file is used to communicate with the host (arduino uno) to compress
% recieve data (namely voltage and force and time) and plot (with filter,
% bias removed, etc)

%client closed loop
function client_CL(port)

%   Example: (You may want to hardcode the port)
%       client('/dev/ttyUSB0') (Linux/Mac)
%       client('COM3') (PC)

% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, {hardware,software,none} flow control
% arduino doesn't have handshaking
% wait up to 30 seconds for data before timing out

mySerial = serial(port, 'BaudRate', 230400, 'FlowControl', 'software','Timeout',30); 
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));                                 

has_quit = false;

% menu loop
while ~has_quit
    fprintf('\n ---------IPMC COMPRESSION INTERFACE-----------------\n');
    fprintf(' ---------------- MOVEMENT --------------------------\n');
    fprintf(' w: Move Up                                  s: Move Down\n');
    fprintf(' a: Set Revs [float, i.e 0.5]                d: Set Speed [50? to 2000?]\n');
    fprintf(' e: Current Revolution and Speed \n');
    fprintf(' ---------------- LOAD CELL -------------------------\n');
    fprintf(' r: Read current lbs                         t: Set Calibration Factor [default: 390000]\n');
    fprintf(' y: Read stream of data \n');
    fprintf(' ---------------- COMPRESSION ------------------------\n');
    fprintf(' f: Test 1: RampUp-SS-RampDown g: h:\n');
    % open scope voltage
    % test #1 ... #n
    % plot data in different ways, w/out bias, filter etc...
    % save data to a folder
    % clean data
    % when i start i want it to run the simuliink script and the test at
    % the same time. and when it done i want to have the ability to save it
    % if i want to and I want it to plot automatically. so when i start
    % both the voltage measurement and the compression is happening at the
    % same time so that way I can put them together easier without having
    % to do werid stuff. bu because the data rates may be different i will
    % have to time it so that they match up. so I won't to be able to take
    % the how long i want to do it and insert it into simulink or I want to
    % be able to take the simulink number and put it into the script. 
    
    % some test that can be done is stead state 3 test in one. steady state
    % value so until it stops changing after compressing it a certain
    % rate. but it can also be the transient affects it has when you start
    % applying the ramp force and then after it reaches steady state
    % undercompression you release in the same manner as you compressed and
    % you get another characteristic of how it went down. 
    % why do i choose the weight i choose as the treshold? because this
    % paper does it... its a medical application dealing with tissue you
    % don't apply that much force. arbitrary. 
    fprintf('  q: Quit client ');                   
   
    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
     
    % send the command to the arduino
    fprintf(mySerial,'%c\n',selection);
    % flushinput(mySerial); %flush input buffer, because arduino
    % will be constantly senning loadcell data. 
    % take the appropriate action
    flushinput(mySerial);
    switch selection
        case 'w' %Move UP
            %fprintf(mySerial, '%c\n', 'w');
            fprintf('Move UP\n');
        case 's' %Move DOWN
            fprintf('Move DOWN\n');
            %fprintf(mySerial, '%c\n', 's');
%             i_mA = fscanf(mySerial, '%f');
%             fprintf('The current in mA is: %f \n\n', i_mA);
%             n = input('PWM duty cycle [-100 (ccw) to 100 (cw)]: '); %in percent 
%             fprintf(mySerial, '%d\n',n); %send the number
        case 'a'
            n = input('float from 0 to 2: ');
            fprintf(mySerial, '%f\n', n);
        case 'd'
            n = input('int from 50 to 2000: ');
            fprintf(mySerial, '%d\n', n);
        case 'e'
            rev = fscanf(mySerial, '%f');
            fprintf('Revolution = %f\n', rev);
            speed = fscanf(mySerial, '%d');
            fprintf('Speed = %d\n', speed);
        case 'r' % wait for a bit on the first initiation
            load = fscanf(mySerial, '%f');
            fprintf('%f\n',load);
        case 't'
            n = input('integer: ');
            fprintf(mySerial, '%d\n', n);
        case 'y'
            % while tic toc less than x seconds. do this
            i = 1;
            while i < 50;
                loadcell = fscanf(mySerial, '%f');
                test(i) = loadcell;
                i = i +1; 
            end
            assignin('base', 'sample_y', test); %base or caller, need to exit function to see it. 
        case 'f'
            % test1_CL(mySerial);
            % General Test
            % ** NOTE ** when changing time, must change the sampling
            % duration in code > ext control pannel > signal & triggering> duration [sec] + 1
            simulation_CL(mySerial, 60, 3); % (serial, time[sec], force [lbs])
            
            %shahinpoor - range was 5g to 50grams
        case 'q' % Quit client
            has_quit = true;             % exit client
%         case 'v' % test
%             fprintf(mySerial, '%c\n', 'w'); % sending like this is not an issue
        otherwise
            fprintf('Invalid Selection %c\n\n', selection);
    end %end switch
end %end while

%got to clear/close all after every run..?
 
end %end function
