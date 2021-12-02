1. See the steps images.
2. In step5, choose the baudrate (Bits per second)
3. In step6->>
	I. Set port value from step4
	II. Set Baud rate from step5
	III. Press Device1 Initialize
	IV. You wil hear the sound of jog movement. If not, wait for 5 sec and press Device1 Initialize again
	V. In the python terminal, it will show that Device1 is initialized.

4. If the jog was not in home position during the power connection:
	I. Then use the jog to make the jog to go to the home position.
	II. Power disconnect and connect with the device
	III. Follw step 3
	
	OR
	I. Press Disconnect Device1 Button
	II. Use the thorlab kinesis software to go home (reset home at the same time)
	III. Disconnect device and close kinesis software
	IV. Follow step 3

5. When initialized, the data is updated automatically. If u want to see the updated position, info
in the setting then press Data Update first, you will hear the sound of jog moving forward and backward
for same distance and data will be updated. Then go to Setting.

6. In the setting option when you press update, it will update the data and in the python terminal 
you will see update confirmation.

7. Any constructive suggestion is appreciated.

8. python version: 3.8

9. Libraries requirement:

Given with pip command:


pip install thorlabs-apt-protocol==25.2.0
pip install pandas==1.3.3
pip install numpy==1.19.2
pip install tkintertable==1.3.3
pip install pyserial==3.5

	




