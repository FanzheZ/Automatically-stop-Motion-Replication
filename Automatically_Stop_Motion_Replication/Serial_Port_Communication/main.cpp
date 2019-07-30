//This code snippet will help you to read data from arduino

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "SerialPort.h"

using std::cout;
using std::endl;

/*Portname must contain these backslashes, and remember to
replace the following com port*/
const char *port_name1 ="\\\\.\\COM10";
const char *port_name2= "\\\\.\\COM11";

//String for incoming data
char incomingData1[MAX_DATA_LENGTH];
char incomingData2[MAX_DATA_LENGTH];


int main()
{
	SerialPort arduino1(port_name1);
	SerialPort arduino2(port_name2);

	bool read_result1;
	bool hasWritten2;

	/*char command= a[MAX_DATA_LENGTH];
	char *command1 = 'L';
	char *command2 = 'S';*/

	if (arduino1.isConnected()) cout << "Coordinator 1 Connection Established" << endl;
	else cout << "ERROR, check Coordinator 1"<<endl;
	if (arduino2.isConnected()) cout << "Coordinator 2 Connection Established" << endl;
	else cout << "ERROR, check Coordinator 2"<<endl;


		while (arduino1.isConnected())
		{
			//Check if data has been read or not
			read_result1 = arduino1.readSerialPort(incomingData1, MAX_DATA_LENGTH);


			if (read_result1 != 0)
			{
				cout << "CAV1: " << endl;
				printf("%s", incomingData1);

				hasWritten2 = arduino2.writeSerialPort(incomingData1, MAX_DATA_LENGTH);
				if (hasWritten2)
				{
					//cout << "CAV2 write successfully." << endl;
				}
		
			}
			else
			{
				//cout << "Coordinator1: Error occured reading data" << endl;
			}
			//wait a bit
			Sleep(1000);
		}


	
}
