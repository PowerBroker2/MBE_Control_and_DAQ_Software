/*See https://www.youtube.com/watch?v=l4372qtZ4dc for directions on installing OpenCV for VisualStudio 2015
See https://stackoverflow.com/questions/16883037/remove-secure-warnings-crt-secure-no-warnings-from-projects-by-default-in-vis for getting rid of fopen() error
*/

#include <Windows.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "SerialPort.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

///////////////////////////////////////////////////////////////////PARAMETERS TO CHANGE
#define MATLAB_FILE_NAME "ServoAngles_C=0.05000_K=2.00000_w=2.00000_u=5.00000_dt=0.01000ms.txt"
#define FISH_COM_PORT "\\\\.\\COM18"
#define SHORE_COM_PORT "\\\\.\\COM11"
#define NUM_SERVOS 2
#define NUM_TIME_STEPS 126
#define SEND_PERIOD 10
#define SAMPLE_FREQUENCY 100 //in Hz  -->  100Hz = 0.01s = 10ms
///////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////DO NOT CHANGE THESE!
#define SOF_BYTE 'h'
#define EOF_BYTE 'p'
#define HEADER_DATA_PACKET_SIZE 5
#define PC_TO_FISH_DATA_PACKET_SIZE (NUM_SERVOS + 5)
#define FISH_TO_PC_DATA_PACKET_SIZE ((NUM_SERVOS * 5) + 10)
#define SHORE_TO_PC_DATA_PACKET_SIZE 5
///////////////////////////////////////////////////////////////////

using namespace std;

//Portname must contain these backslashes, and remember to
//replace the following com port
//USB comms for on board Arduino
char *OnBoard_port_name = FISH_COM_PORT;
SerialPort OnBoardArduino(OnBoard_port_name); //115200 Baud

//USB comms for shore DAQ Arduino
//char *Shore_port_name = SHORE_COM_PORT;
//SerialPort ShoreArduino(Shore_port_name); //115200 Baud

struct servoMeasurements
{
	byte angle[NUM_SERVOS];
	int16_t angularVelocity[NUM_SERVOS];
	uint16_t torque[NUM_SERVOS];
};

servoMeasurements servo_Measurements;

//initialize DAQ result file
ofstream FishDAQ("FishDAQ.txt");
ofstream ShoreDAQ("ShoreDAQ.txt");

//initialize FILE stream to access .txt file 
FILE * pFile;

//initialize input file stream variable for MATLAB outputs
ifstream inFile;

//declare functions
int getNumTimesteps(ifstream& inFile);
int getSendPeriod(ifstream& inFile);
int getNumServos(ifstream& inFile);
void getServoData(ifstream& inFile, char servoPositionArray[]);
void connectToArduinos();
void sendDataPacket(char PCFishDataPacket[], int timeStepIndex, char servoPositionArray[]);
int getFishData(char Packet[], uint16_t &instPower, unsigned long &totalSampleTimestamp);
void recordFishData(unsigned long i, uint16_t instPower, unsigned long totalSampleTimestamp);



void main()
{
	//arrays for serial packets to and from the Arduinos
	char PCFishDataPacket[PC_TO_FISH_DATA_PACKET_SIZE];
	char FishToPCDataPacket[FISH_TO_PC_DATA_PACKET_SIZE];
	char ShoreToPCDataPacket[SHORE_TO_PC_DATA_PACKET_SIZE];
	
	//array to hold data from MATLAB text file
	char servoPositionArray[NUM_TIME_STEPS * NUM_SERVOS];

	//buffer for grabbing data from MATLAB text file
	string textInputBuff;
	int numTimeSteps = 0;
	int sendPeriod = 0;
	int numServos = 0;
	int length = 0;

	//sample and timestep index
	unsigned long sampleIndex = 0;
	int timeStepIndex = 0;

	//instantaneous power
	uint16_t instPower = 0;

	//sample time in ms
	unsigned long totalSampleTimestamp = 0;



	//initialize fish DAQ text file
	pFile = fopen("FishDAQ.txt", "a+");
	fprintf(pFile, "Fish DAQ log \n\nSample index  total instantaneous power  servo 1 angle  servo 1 angular velocity  servo 1 torque  -->(repeat for all servos...)  time since LED on (ms) \n\n");
	fclose(pFile);

	//initialize shore DAQ text file
	pFile = fopen("ShoreDAQ.txt", "a+");
	fprintf(pFile, "Shore DAQ log \n\nSample index  total instantaneous power  servo 1 angle  servo 1 angular velocity  servo 1 torque  -->(repeat for all servos...)  time since LED on (ms) \n\n");
	fclose(pFile);



	//open MATLAB output text file
	inFile.open(MATLAB_FILE_NAME);
	if (!inFile)
	{
		cout << "Unable to open MATLAB output file";
		while (1);
	}



	//get number of time steps
	numTimeSteps = getNumTimesteps(inFile);
	
	//get send period
	sendPeriod = getSendPeriod(inFile);

	//get number of servos
	numServos = getNumServos(inFile);

	//get everything else
	getServoData(inFile, servoPositionArray);

	//wait for Arduinos to connect and come online
	connectToArduinos();



	//send all servo angles to fish MCU
	while (timeStepIndex < NUM_TIME_STEPS)
	{
		//send data
		sendDataPacket(PCFishDataPacket, timeStepIndex, servoPositionArray);
		
		//increment
		timeStepIndex = timeStepIndex + 1;

		//wait for MCU to "catch up" before next transmission
		Sleep(10);
	}



	//get telem from fish and shore MCUs
	while (true)
	{
		//fetch telemetry from fish
		getFishData(FishToPCDataPacket, instPower, totalSampleTimestamp);

		//record it in a .txt file
		recordFishData(sampleIndex, instPower, totalSampleTimestamp);

		//increment sample index
		sampleIndex = sampleIndex + 1;
	}
}



int getNumTimesteps(ifstream& inFile)
{
	string textInputBuff;
	int length = 0;
	int numTimeSteps = 0;
	int index = 0;
	int newIndex = 0;

	getline(inFile, textInputBuff);
	length = textInputBuff.length();
	cout << "Number of timesteps: ";
	index = 0;
	while (index < length)
	{
		index = index + 1;
	}
	index = index - 2;
	newIndex = 0;
	while (index >= 0)
	{
		if (!newIndex)
		{
			numTimeSteps = numTimeSteps + (textInputBuff[index] - '0');
		}
		else
		{
			numTimeSteps = numTimeSteps + ((textInputBuff[index] - '0') * pow(10, newIndex));
		}
		index = index - 1;
		newIndex = newIndex + 1;
	}
	cout << numTimeSteps << endl;

	if (numTimeSteps != NUM_TIME_STEPS)
	{
		cout << "ERROR - WRONG TIME STEP";
		while (1);
	}

	return numTimeSteps;
}



int getSendPeriod(ifstream& inFile)
{
	string textInputBuff;
	int length = 0;
	int sendPeriod = 0;
	int index = 0;
	int newIndex = 0;

	getline(inFile, textInputBuff);
	length = textInputBuff.length();
	cout << "Send period (in ms): ";
	index = 0;
	while (index < length)
	{
		index = index + 1;
	}
	index = index - 2;
	newIndex = 0;
	while (index >= 0)
	{
		if (!newIndex)
		{
			sendPeriod = sendPeriod + (textInputBuff[index] - '0');
		}
		else
		{
			sendPeriod = sendPeriod + ((textInputBuff[index] - '0') * 10 * newIndex);
		}
		index = index - 1;
		newIndex = newIndex + 1;
	}
	cout << sendPeriod << endl;

	if (sendPeriod != SEND_PERIOD)
	{
		cout << "ERROR - WRONG SEND PERIOD";
		while (1);
	}

	return sendPeriod;
}



int getNumServos(ifstream& inFile)
{
	string textInputBuff;
	int length = 0;
	int numServos = 0;
	int index = 0;
	int newIndex = 0;

	getline(inFile, textInputBuff);
	length = textInputBuff.length();
	cout << "Number of servos: ";
	index = 0;
	while (index < length)
	{
		index = index + 1;
	}
	index = index - 2;
	newIndex = 0;
	while (index >= 0)
	{
		if (!newIndex)
		{
			numServos = numServos + (textInputBuff[index] - '0');
		}
		else
		{
			numServos = numServos + ((textInputBuff[index] - '0') * 10 * newIndex);
		}
		index = index - 1;
		newIndex = newIndex + 1;
	}
	cout << numServos << endl;

	if (numServos != NUM_SERVOS)
	{
		cout << "ERROR - WRONG SERVO NUMBER";
		while (1);
	}

	return numServos;
}



void getServoData(ifstream& inFile, char servoPositionArray[])
{
	string textInputBuff; //buffer string
	int length = 0;     //number of chars in the current line of text from MATLAB file
	int arrayIndex = 0; //index counter for buffer array
	int timeIndex = 1; //index for current time step
	int servoIndex = 1; //index for current servo
	int outputIndex = 0; //index for output array

	//get first line of servo angles
	getline(inFile, textInputBuff);
	length = textInputBuff.length();

	//loop through all servo angles for all time steps
	while (timeIndex <= NUM_TIME_STEPS)
	{
		servoIndex = 1;
		while ((servoIndex <= NUM_SERVOS) && ((arrayIndex + 3) < length))
		{
			//convert and store new angle
			servoPositionArray[outputIndex] = (100 * (textInputBuff[arrayIndex] - '0')) + (10 * (textInputBuff[arrayIndex + 1] - '0')) + (textInputBuff[arrayIndex + 2] - '0');
			
			//update output array index counter
			outputIndex = outputIndex + 1;

			//increment buffer index
			arrayIndex = arrayIndex + 4;

			//increment servo counter
			servoIndex = servoIndex + 1;
		}

		//get new data from next line
		getline(inFile, textInputBuff);
		length = textInputBuff.length();

		//increment time index
		timeIndex = timeIndex + 1;

		//reset array index
		arrayIndex = 0;
	}
}



void sendDataPacket(char PCFishDataPacket[], int timeStepIndex, char servoPositionArray[])
{
	int i = 4;

	//make data packet
	PCFishDataPacket[0] = SOF_BYTE;
	PCFishDataPacket[1] = NUM_TIME_STEPS;
	PCFishDataPacket[2] = SEND_PERIOD;
	PCFishDataPacket[3] = NUM_SERVOS;

	while ((i - 4) < NUM_SERVOS)
	{
		PCFishDataPacket[i] = servoPositionArray[timeStepIndex * NUM_SERVOS + (i - 4)];

		i = i + 1;
	}

	PCFishDataPacket[PC_TO_FISH_DATA_PACKET_SIZE - 1] = EOF_BYTE;


	//send packet
	if (OnBoardArduino.isConnected())
	{
		//send data
		OnBoardArduino.writeSerialPort(PCFishDataPacket, PC_TO_FISH_DATA_PACKET_SIZE);
	}
	else
	{
		cout << "ERROR, serial loss link - Fish" << endl;
		while (1);
	}
}



void connectToArduinos()
{
	//wait for connection to both Arduinos and camera
	if (!OnBoardArduino.isConnected())
	{
		cout << "ERROR - on board Arduino not connected!" << endl;
		while (1);
	}
	/*if(!ShoreArduino.isConnected())
	{
	cout << "ERROR - shore Arduino not connected!" << endl;
	while(1);
	}*/

	/*VideoCapture cap(1); //capture the video from webcam
	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;

		//wait for cam
		while (!cap.isOpened())
		{
		//do nothing
		}
	}*/
}



int getFishData(char FishToPCDataPacket[], uint16_t &instPower, unsigned long &totalSampleTimestamp)
{
	int readSuccessful = 0; //determine if read from Arduino was successful
	int i = 0; //servo index counter

	//read the data if connected
	if (OnBoardArduino.isConnected())
	{
		//read data
		readSuccessful = OnBoardArduino.readSerialPort(FishToPCDataPacket, FISH_TO_PC_DATA_PACKET_SIZE);
	}
	else
	{
		cout << "ERROR, serial loss link - Fish" << endl;
		while (1);
	}

	if (readSuccessful && (FishToPCDataPacket[0] == SOF_BYTE) && (FishToPCDataPacket[FISH_TO_PC_DATA_PACKET_SIZE - 1] == EOF_BYTE))
	{
		instPower = 0;
		instPower = ((unsigned char)FishToPCDataPacket[1]) << 8;
		instPower = instPower | ((unsigned char)FishToPCDataPacket[2]);
		//printf("%i %u ", readSuccessful, instPower);

		while (i < NUM_SERVOS)
		{
			servo_Measurements.angle[i] = 0;
			servo_Measurements.angle[i] = ((unsigned char)FishToPCDataPacket[(i * 5) + 3]);

			servo_Measurements.angularVelocity[i] = 0;
			servo_Measurements.angularVelocity[i] = ((unsigned char)FishToPCDataPacket[(i * 5) + 4]) << 8;
			servo_Measurements.angularVelocity[i] = servo_Measurements.angularVelocity[i] | ((unsigned char)FishToPCDataPacket[(i * 5) + 5]);
			servo_Measurements.angularVelocity[i] = servo_Measurements.angularVelocity[i] * SAMPLE_FREQUENCY;

			servo_Measurements.torque[i] = 0;
			servo_Measurements.torque[i] = ((unsigned char)FishToPCDataPacket[(i * 5) + 6]) << 8;
			servo_Measurements.torque[i] = servo_Measurements.torque[i] | ((unsigned char)FishToPCDataPacket[(i * 5) + 7]);

			//printf("%u %u %u ", servo_Measurements.angle[i], servo_Measurements.angularVelocity[i], servo_Measurements.torque[i]);

			i = i + 1;
		}

		totalSampleTimestamp = 0;
		totalSampleTimestamp = ((unsigned char)FishToPCDataPacket[FISH_TO_PC_DATA_PACKET_SIZE - 7]) << (5 * 8);
		totalSampleTimestamp = totalSampleTimestamp | (((unsigned char)FishToPCDataPacket[FISH_TO_PC_DATA_PACKET_SIZE - 6]) << (4 * 8));
		totalSampleTimestamp = totalSampleTimestamp | (((unsigned char)FishToPCDataPacket[FISH_TO_PC_DATA_PACKET_SIZE - 5]) << (3 * 8));
		totalSampleTimestamp = totalSampleTimestamp | (((unsigned char)FishToPCDataPacket[FISH_TO_PC_DATA_PACKET_SIZE - 4]) << (2 * 8));
		totalSampleTimestamp = totalSampleTimestamp | (((unsigned char)FishToPCDataPacket[FISH_TO_PC_DATA_PACKET_SIZE - 3]) << (1 * 8));
		totalSampleTimestamp = totalSampleTimestamp | ((unsigned char)FishToPCDataPacket[FISH_TO_PC_DATA_PACKET_SIZE - 2]);

		//printf("%u\n", totalSampleTimestamp);
	}
	return readSuccessful;
}



void recordFishData(unsigned long i, uint16_t instPower, unsigned long totalSampleTimestamp)
{
	int servoIndex = 0;

	pFile = fopen("FishDAQ.txt","a+");
	fprintf(pFile, "%u %u ", i, instPower);

	while (servoIndex < NUM_SERVOS)
	{
		fprintf(pFile, "%u %u %u ", servo_Measurements.angle[servoIndex], servo_Measurements.angularVelocity[servoIndex], servo_Measurements.torque[servoIndex]);

		servoIndex = servoIndex + 1;
	}

	fprintf(pFile, "%u\n", totalSampleTimestamp);
	fclose(pFile);
}