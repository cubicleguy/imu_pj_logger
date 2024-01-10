HW PREREQUISTES
===============
1. PCAN-USB Adapter or compatible
2. Epson IMU CAN J1939 compatible Device 


SW PREREQUISTES
===============
1. Linux kernel, v5.4 or higher, that supports the J1939 stack 
*(The software was tested with Ubuntu 20.04, kernel 5.13.0-35-generic)


CREATE CAN NETWORK
==================
Add device:
sudo ip link add dev can0 type can

Set default can bitrate:
sudo ip link set can0 type can bitrate 250000

Set it up:
sudo ip link set up can0

If you want to monitor CAN network:
candump can0

Please note that if you meet a network problem in a future you can try make it down and then up again:
sudo ip link set down can0
sudo ip link set up can0


BUILD IMU_PJ_LOGGER
======================
1. Go to 'imu_pj_logger' and compile:

   gcc main.c -o imu_pj_logger -lpthread -Wall

2. If everything is right and devices are in sample mode, then run imu_pj_logger with required ID nodes(ctrl+c to exit):

   ./imu_pj_logger --id 83 81

   The output on terminal should look like:

	* dev addr:83h, name:G552PJ, version:100700, serial:00000001
	* samples rate:25 Sps
	* filter:Moving average filter TAP=128
	* auto start:1, addr:83h, rate:250kbps
	* priority response:6, sout:6
	* enabled sout:1245

	* dev addr:81h, name:G552PJ, version:100100, serial:00000009
	* samples rate:200 Sps
	* filter:FIR Kaiser filter TAP=32 and fc=50Hz
	* attitude mode:Euler, reference:0h, motion:3 m/s
	* auto start:1, addr:81h, rate:250kbps
	* priority response:6, sout:6
	* enabled sout:12457

	...
	Id:83 Sw:2515 Hw:203 S:0.2000 Gx:+0.03030396 Gy:+0.01513672 Gz:+0.00000000 Ax:-5.20019531 Ay:+4.79980469 Az:+1002.79980469 Ts:+27.95760345 Timestamp:1647885820.419
	Id:81 Sw:19005 Hw:254 S:0.2650 Gx:+0.07574463 Gy:+0.15151978 Gz:+0.01513672 Ax:+14.00000000 Ay:+6.40039062 Az:+1004.00000000 Ts:+28.78421593 Timestamp:1647885816.806 a1:-0.49002075 a2:-0.60000610
	Id:81 Sw:19006 Hw:255 S:0.2700 Gx:+0.03030396 Gy:-0.10604858 Gz:+0.06060791 Ax:+16.79980469 Ay:+6.79980469 Az:+1003.60058594 Ts:+28.78800774 Timestamp:1647885816.886 a1:-0.49002075 a2:-0.60000610
	Id:81 Sw:19007 Hw:  0 S:0.2750 Gx:+0.06060791 Gy:-0.09091187 Gz:+0.00000000 Ax:+15.60058594 Ay:+5.20019531 Az:+1002.00000000 Ts:+28.78800774 Timestamp:1647885816.966 a1:-0.49002075 a2:-0.60000610 
	Id:81 Sw:19008 Hw:  1 S:0.2800 Gx:+0.07574463 Gy:-0.03030396 Gz:+0.03030396 Ax:+18.40039062 Ay:+4.79980469 Az:+1004.79980469 Ts:+28.78800774 Timestamp:1647885817.046 a1:-0.49002075 a2:-0.60000610  
	...

3. If there are no samples output then try set devices in sample mode:

	./imu_pj_logger --id 83 81 --mode smpl

	And then to get samples:

	./imu_pj_logger --id 83 81


THE COMMANDS
============
1. mandatory command:

	--id

	This command is made mandatory command to distinguish Epson devices from others that can be in CAN.
	This command can have multiple arguments (ID):
	--id 80 83 a1

2. The commands that cannot be run at the same time(only one of them at a time):

   --info
   --read
   --write 
   --msc
   --cfg1
   --cfg2
   --mode
   --dump
   --err

   In case of an attempt to run two or more of those commands the only last one is performed.
   For example, this run reads only register 0x00 and dump has no effect:

   ./imu_pj_logger --id 80 --dump --read 0

3. The commands that should have only one parameter at a time:

	--msc
	--mode

	For example, the right ones are "--mode smpl" or "--mode cfg" or "--mode rst", but not "--mode rst smpl"

4. The commands that should have:
	a). should have only one device ID number
	b). requires a device to be in configuration mode
	a). can have one or more parameters with arguments 

	--id
	--cfg1
	--cfg2

	For example, for a device with ID 80:
		a). set new ID to A1
		b). enable SOUT 1, 2, and 4 (disable 7)
		c). enable auto start

   ./imu_pj_logger --id 80 --cfg1 newid:a1 souten:124 autostart:1

	Now you can save the changes:
	./imu_pj_logger --id 80 --msc save

	Reset the device:
	./imu_pj_logger --id 80 --mode rst

	And get 2 samples from the device with new ID A1:
	* dev addr:a1h, name:G552PJ, version:100100, serial:00000009
	* samples rate:200 Sps
	* filter:FIR Kaiser filter TAP=32 and fc=50Hz
	* attitude mode:Euler, reference:0h, motion:3 m/s
	* auto start:1, addr:a1h, rate:250kbps
	* priority response:6, sout:6
	* enabled sout:12457

	Id:a1 Sw:  0 Hw:211 S:0.0000 Gx:+0.15151978 Gy:-0.10604858 Gz:+0.01513672 Ax:+11.00000000 Ay:+2.79980469 Az:+1003.20019531 Ts:+27.35470772 
	Id:a1 Sw:  1 Hw:212 S:0.0050 Gx:+0.06060791 Gy:+0.28787231 Gz:-0.03030396 Ax:+12.79980469 Ay:+6.79980469 Az:+1002.79980469 Ts:+27.35470772

   
USAGE CASES
===========
1. Set device(s) in configuration mode to change ID, sample rate, filter and other settings described in --cfg1 and --cfg2 commands.
   For example,
	a). set a device with ID 80 in configuration mode:
		./imu_pj_logger --id 80 --mode cfg
	b). set two devices on the CAN in configuration mode:
		./imu_pj_logger --id 80 81 --mode cfg

2. Make the changes for each device by a separate command:
	./imu_pj_logger --id 80 --cfg1 newid:a1 souten:1245 autostart:1
	./imu_pj_logger --id 81 --cfg1 newid:a2 souten:12457 autostart:1

3. Save changes in the devices:
	./imu_pj_logger --id 80 81 --msc save

4.	Reset the devices:
	./imu_pj_logger --id 80 81 --mode rst

5. Start gathering samples from the devices (ctrl+c to exit):
	./imu_pj_logger --id 80 81 

6. Get 1000 samples from the devices and then exit:
	./imu_pj_logger --id 80 81  -x 1000

7. Get 1000 samples into display and log file(s) from the devices and then exit:
	./imu_pj_logger --id 80 81  -x 1000 --log

8. Get 90000 samples into display and log file(s) from the devices with timestamp update from host every 60 seconds and then exit:
	./imu_pj_logger --id 80 81  -x 90000 --log --time 60

9. Error flag is recorded in separate file(s) while sampling. After sampling you can check more details of errors by --err command:
	./imu_pj_logger --id 80 81 --err

	*if there is no output then no error detected.

10. Dump registers from the devices:
	./imu_pj_logger --id 80 81  --dump




