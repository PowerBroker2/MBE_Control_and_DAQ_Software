#include <Windows.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "SerialPort.h"

using namespace std;

#define DATA_FILE_NAME "Test_1.txt" //"Prototype1_FlexPanel1_Amplidude_4deg_Frequency_3Hz_Trial_1.txt"
#define MCU_COM_PORT "\\\\.\\COM18"
//#define DATA_PACKET_SIZE 8
#define DATA_PACKET_SIZE 6
#define SOF_BYTE 'h'
#define EOF_BYTE 'p'

//initialize DAQ result file
ofstream FishDAQ(DATA_FILE_NAME);
//initialize FILE stream to access .txt file 
FILE * pFile;

//Portname must contain these backslashes, and remember to
//replace the following com port
//USB comms for on board Arduino
char *MCU_port_name = MCU_COM_PORT;
SerialPort MCU(MCU_port_name); //115200 Baud

void main()
{
	//array for serial packets from the MCU
	char data[DATA_PACKET_SIZE] = {0};
	//result from torque measurements
	uint16_t torque = 0;
	uint16_t feedback = 0;
	uint16_t servoInput = 0;

	//determine if read from Arduino was successful
	int readSuccessful = 0;

	//initialize fish DAQ text file
	pFile = fopen(DATA_FILE_NAME, "a+");

	//connect to MCU
	if (!MCU.isConnected())
	{
		cout << "ERROR - on board Arduino not connected!" << endl;
		while (1);
	}

	while (1)
	{
		if (MCU.isConnected())
		{
			//read data
			readSuccessful = MCU.readSerialPort(data, DATA_PACKET_SIZE);
		}
		else
		{
			cout << "ERROR, serial loss link - Fish" << endl;
			while (1);
		}

		if (readSuccessful && (data[0] == SOF_BYTE) && (data[DATA_PACKET_SIZE - 1] == EOF_BYTE))
		{
			torque = ((byte)data[1] << 8) | (byte)data[2];
			feedback = ((byte)data[3] << 8) | (byte)data[4];

			//printf("torque: %u  feedback: %u  input: %u\n", torque, feedback, servoInput);

			//write data if data was received
			pFile = fopen(DATA_FILE_NAME, "a+");
			fprintf(pFile, "%u\n%u\n", torque, feedback);
			//fprintf(pFile, "%u\n%u\n%u\n", torque, servoInput, feedback);
			fclose(pFile);
		}
	}
}
