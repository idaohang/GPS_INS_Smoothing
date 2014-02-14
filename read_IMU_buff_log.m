function [buffer] = read_IMU_buff_log(buff_log, start_time, end_time)
if nargin < 2
	start_time = 0;
    end_time = 1000;
else if nargin < 3
        end_time = 1000;
    end
end

i = 1;
% push buffer from the starting time
while ( buff_log(i,1)<start_time )
    i=i+1;
end
j=i+1;
while ( buff_log(j,1)<end_time )
    j=j+1;
end
buff_log = buff_log(i:j,:);
buff_size = length( buff_log(:,1) );
meas_temp.gyro_meas = zeros(3,1);
meas_temp.accel_meas = zeros(3,1);

for index = 1:buff_size
    data = buff_log(index,:);
    count = 1;
    meas_temp.imu_tm = data(count);
    count=count+1;
    meas_temp.dt = data(count);
    count=count+1;
    meas_temp.gyro_meas(1) = data(count);
    count=count+1;
    meas_temp.gyro_meas(2) = data(count);
    count=count+1;
    meas_temp.gyro_meas(3) = data(count);
    count=count+1;
    meas_temp.accel_meas(1) = data(count);
    count=count+1;
    meas_temp.accel_meas(2) = data(count);
    count=count+1;
    meas_temp.accel_meas(3) = data(count);
    count=count+1;

    if index == 1
        buffer(buff_size,1)=meas_temp;
        buffer(1)=meas_temp;
    else
        buffer(index)=meas_temp;
    end
end


