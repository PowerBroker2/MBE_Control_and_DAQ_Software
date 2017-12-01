clear
clc

servoSpeedRating = .05; %time in seconds to travel 60 degrees
minServoThrow = 36;
maxServoThrow = 144;

C = 0.05; %wave amplitude in m (limited to  less than peduncle length if not using exponential sinusoid)
L = 2; %expenential slope constant in 1/m (larger k means exponential increases faster for a given change in x)
k = 1; %spacial frequency in rad/m
w = 5; %phase velocity in rad/s
samplePeriod = 0.01; %sample period in s

x = 0:0.001:1; %distance from head in m
z = linspace(-pi/2,pi/2,1000); %array only used in plotting semicircles

peduncle_length = 0.1; %peduncle length in m
tolerance = 0.001; %tolerance for calculating intersection values
numServos = 2; %number of fish tail segments
numTimeSteps = ceil((2*pi/w)/samplePeriod);

e = C.*(exp(L.*x) - 1); %envelope of wave (if exponential is used)
%plot(x,e);

timeIndex = 1; %keeps track of which time-step the intersections/servo angles are being calculated at

for t = 0:samplePeriod:2*pi/w %increase time in steps of 10ms
    y = C.*(1).*sin(k.*x - w.*t); 
    %y = C.*(exp(K.*x) - 1).*sin(w.*x - u.*t);
    hold all
    plot(x,y);
    grid on
    title('Fish Tail Shape');
    xlabel('Distance from Fish Head (m)');
    ylabel('Distance from Fish Head (m)');

    %array to hold x and y coordinates of each servo head at a given time
    servoNode_X(1) = 0;
    servoNode_Y(1) = 0;

    hold all
    plot(peduncle_length.*cos(z)+servoNode_X(1),peduncle_length.*sin(z)+servoNode_Y(1))

    %create semicircle centered at the previous servo head's x and y
    %coordinates
    y2 = real(sqrt(peduncle_length.^2 - (x).^2));
    y3 = real(-sqrt(peduncle_length.^2 - (x).^2));

    %calculate all possible intersections
    [xout,yout] = intersections(x,y,x,y2,1);
    [xout2,yout2] = intersections(x,y,x,y3,1);
    
    %find the correct intersection point
    k = 1;
    trigger = 1;
    while k <= length(xout) && trigger
        if xout(k) > servoNode_X(1) && (sqrt((xout(k) - servoNode_X(1)).^2 + (yout(k) - servoNode_Y(1))^2)) <= (peduncle_length + tolerance)
            trigger = 0;

            servoNode_X(2) = xout(k);
            servoNode_Y(2) = yout(k);
        end

        k = k + 1;
    end

    j = 1;
    if trigger
        while j <= length(xout2) && trigger
            if xout2(j) > servoNode_X(1) && (sqrt((xout2(j) - servoNode_X(1)).^2 + (yout2(j) - servoNode_Y(1))^2)) <= (peduncle_length + tolerance)
                trigger = 0;

                servoNode_X(2) = xout2(j);
                servoNode_Y(2) = yout2(j);
            end

            j = j + 1;
        end
    end

    %create circle centered at the previous servo head's x and y
    %coordinates
    y2 = real(sqrt(peduncle_length.^2 - (x - servoNode_X(2)).^2) + servoNode_Y(2));
    y3 = real(-sqrt(peduncle_length.^2 - (x - servoNode_X(2)).^2) + servoNode_Y(2));

    hold all
    plot(servoNode_X(2),servoNode_Y(2),'r.','markersize',18)
    hold all
    plot(peduncle_length.*cos(z)+servoNode_X(2),peduncle_length.*sin(z)+servoNode_Y(2))

    %initialize the running sum of servo angles
    cumulativeServoAngle = 0;
    %calculate servo angle for a given time for the first servo and then
    %round to nearest integer
    servoAngle(timeIndex,1) = atand((servoNode_Y(2)-servoNode_Y(1))/(servoNode_X(2)-servoNode_X(1))) + 90;
    servoAngle(timeIndex,1) = round(servoAngle(timeIndex,1));
    %update the running sum of servo angles
    cumulativeServoAngle = cumulativeServoAngle + (servoAngle(timeIndex,1) - 90);
    
    %initilize fish tail segment index
    i = 3;
    %loop to find servo angle for every remaining segment
    while i <= numServos

        %calculate all possible intersections
        [xout,yout] = intersections(x,y,x,y2,1);
        [xout2,yout2] = intersections(x,y,x,y3,1);

        %find the correct intersection point
        k = 1;
        trigger = 1;
        while k <= length(xout) && trigger
            if xout(k) > servoNode_X(i-1) && (sqrt((xout(k) - servoNode_X(i-1)).^2 + (yout(k) - servoNode_Y(i-1))^2)) <= (peduncle_length + tolerance)
                trigger = 0;

                servoNode_X(i) = xout(k);
                servoNode_Y(i) = yout(k);
            end

            k = k + 1;
        end

        j = 1;
        if trigger
            while j <= length(xout2) && trigger
                if xout2(j) > servoNode_X(i-1) && (sqrt((xout2(j) - servoNode_X(i-1)).^2 + (yout2(j) - servoNode_Y(i-1))^2)) <= (peduncle_length + tolerance)
                    trigger = 0;

                    servoNode_X(i) = xout2(j);
                    servoNode_Y(i) = yout2(j);
                end

                j = j + 1;
            end
        end

        hold all
        plot(servoNode_X(i),servoNode_Y(i),'r.','markersize',18)
        hold all
        plot(peduncle_length.*cos(z)+servoNode_X(i),peduncle_length.*sin(z)+servoNode_Y(i))

        %calculate servo angle for a given time for the given servo and then
        %round to nearest integer
        servoAngle(timeIndex,i-1) = (atand((servoNode_Y(i)-servoNode_Y(i-1))/(servoNode_X(i)-servoNode_X(i-1))) + 90) - cumulativeServoAngle;
        servoAngle(timeIndex,i-1) = round(servoAngle(timeIndex,i-1));
        %update the running sum of servo angles
        cumulativeServoAngle = cumulativeServoAngle + (servoAngle(timeIndex,i-1) - 90);
        
        %create circle centered at the previous servo head's x and y
        %coordinates
        y2 = real(sqrt(peduncle_length.^2 - (x - servoNode_X(i)).^2) + servoNode_Y(i));
        y3 = real(-sqrt(peduncle_length.^2 - (x - servoNode_X(i)).^2) + servoNode_Y(i));

        i = i + 1;
    end


    %calculate all possible intersections
    [xout,yout] = intersections(x,y,x,y2,1);
    [xout2,yout2] = intersections(x,y,x,y3,1);

    %find the correct intersection point
    k = 1;
    trigger = 1;
    while k <= length(xout) && trigger
        if xout(k) > servoNode_X(i-1) && (sqrt((xout(k) - servoNode_X(i-1)).^2 + (yout(k) - servoNode_Y(i-1))^2)) <= (peduncle_length + tolerance)
                trigger = 0;

            servoNode_X(i) = xout(k);
            servoNode_Y(i) = yout(k);
        end

        k = k + 1;
    end

    j = 1;
    if trigger
        while j <= length(xout2) && trigger
            if xout2(j) > servoNode_X(i-1) && (sqrt((xout2(j) - servoNode_X(i-1)).^2 + (yout2(j) - servoNode_Y(i-1))^2)) <= (peduncle_length + tolerance)
                trigger = 0;

                servoNode_X(i) = xout2(j);
                servoNode_Y(i) = yout2(j);
            end

            j = j + 1;
        end
    end

    hold all
    plot(servoNode_X(i),servoNode_Y(i),'r.','markersize',18)

    %calculate servo angle for a given time for the given servo and then
    %round to nearest integer
    servoAngle(timeIndex,i-1) = (atand((servoNode_Y(i)-servoNode_Y(i-1))/(servoNode_X(i)-servoNode_X(i-1))) + 90) - cumulativeServoAngle;
    servoAngle(timeIndex,i-1) = round(servoAngle(timeIndex,i-1));
    
    %increment the time index
    timeIndex = timeIndex + 1;
end


%import to simulink model
%----------------------------------------------------------------------------------------------------------
%test to see if any angle changes exceed maximum servo speed
servoThrowTimeConstant = servoSpeedRating/60; %seconds per degree (change only numerator)
servoIndex = 1;
while servoIndex <= numServos
    timeIndex = 1;
    
    while timeIndex < numTimeSteps
        angularDifference = abs(servoAngle(timeIndex + 1,servoIndex) - servoAngle(timeIndex,servoIndex));
        throwTime = angularDifference * servoThrowTimeConstant;
        
        if throwTime >= samplePeriod
            error('ERROR:\nservo angular velocity exceeds max servo speed for servo %i between time steps %i and %i', servoIndex, timeIndex, timeIndex + 1);
        end
        
        if servoAngle(timeIndex,servoIndex) < minServoThrow || servoAngle(timeIndex,servoIndex) > maxServoThrow
            error('ERROR:\nservo angle exceeds max servo angle for servo %i at time step %i', servoIndex, timeIndex);
        end
        
        timeIndex = timeIndex + 1;
    end
    
    servoIndex = servoIndex + 1;
end

%create and write servo angles to a text file used by C++ code to control
%robot
fid = fopen( sprintf('ServoAngles_C=%2.5f_K=%2.5f_w=%2.5f_u=%2.5f_dt=%2.5fms.txt', C, L, k, w, samplePeriod), 'wt' );

fprintf( fid, '%i \n', numTimeSteps);
fprintf( fid, '%i \n', samplePeriod*1000); %record sample period in ms
fprintf( fid, '%i \n', numServos); %record number of servos

timeIndexText = 1;
while timeIndexText <= numTimeSteps
    servoIndexText = 1;
    while servoIndexText <= numServos
        fprintf( fid, '%03d', servoAngle(timeIndexText,servoIndexText));
        fprintf( fid, " ");
        
        servoIndexText = servoIndexText + 1;
    end
    fprintf( fid, "\n");
    
    timeIndexText = timeIndexText + 1;
end

fclose(fid);
%----------------------------------------------------------------------------------------------------------




