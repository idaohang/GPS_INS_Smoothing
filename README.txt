This set of matlab scripts are for GPSINS post-processing with true data. 
Written by Yiming Chen, yimingchen86@gmail.com

Run GPS_INS_Smoothing.m.

In gps_smth_ins function, you can use EKF or MAP smoother as the estimator. 

For both estimator, you can choose use dual frequency measurements or L1 only.

For EKF, double differenced code meas and/or integer solved phase meas can be used.

For smoothing, EKF (only used double differenced code meas) is first applied for initialization. 

In smoothing, only initial condition, INS residual, dd code residual and 
integer free phase residual (see Zhao and Chen's IROS 2014 paper) are used to form the cost function.
No integer resolutions.

Data sets can be found in folder Data_set. Currently, there are 1 stationary data collected with Ant 1
on UCR campus; three on vehicle data around CECERT campus and UCR campus. All the differential messages
are from UCR Ntrip base (ntrip.engr.ucr.edu) with UCR Ant 2.

Ground truth is from the python smoothing code written by Dr. Ahn Vu. Since currently we are not sure
what time tag (pps, imu or system) he used in the python smoothing. Thus, except for the plotting 
the decent error comparison with  statistics is not available temporarily.

