
preprocessed_v1  -   data generated with python, on its turn it reads raw data compiled from rosbags.

preprocessed_v1_1  -   same as preprocessed_v1 but observations have less tight start and end times by 1.5 secs.

preprocessed_v1_2  -   same as preprocessed_v2 buthave more annotation on decision time, handle assignments, outcome, initial orientation of the tray etc.


preprocessed_v2  -   preprocessed_v1 data that is preprocessed and filtered with MATLAB.

preprocessed_v2_1  -   same as preprocessed_v2, however it used preprocessed_v1_1 to perform signal processing. Also, it includes orientation tracking from IMU.

preprocessed_v2_2  -   same as preprocessed_v2_1, however it used preprocessed_v1_2 to perform signal processing. Also, it includes orientation tracking from IMU.



