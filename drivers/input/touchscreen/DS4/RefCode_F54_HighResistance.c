/*
   +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   Copyright (c) 2011 Synaptics, Inc.
   
   Permission is hereby granted, free of charge, to any person obtaining a copy of 
   this software and associated documentation files (the "Software"), to deal in 
   the Software without restriction, including without limitation the rights to use, 
   copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
   Software, and to permit persons to whom the Software is furnished to do so, 
   subject to the following conditions:
   
   The above copyright notice and this permission notice shall be included in all 
   copies or substantial portions of the Software.
   
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
   SOFTWARE.
 */

#include "RefCode.h"
#include "RefCode_PDTScan.h"

#ifdef _F54_TEST_
unsigned char F54_HighResistance(void)
{
	unsigned char imageBuffer[6];
	short resistance[3];
	int i, Result=0;
	unsigned char command;
	int read_count = 0;

#ifdef F54_Porting
	int resistanceLimit[3][2] = { {-1000, 450}, {-1000, 450}, {-400, 20} };	//base value * 1000
	char buf[512] = {0};
	int ret = 0;
#else
	float resistanceLimit[3][2] = {-1, 0.45, -1, 0.45, -0.4, 0.02};
#endif

#ifdef F54_Porting
	ret += sprintf(buf+ret, "\n=====================================================\n");
	ret += sprintf(buf+ret, "\tHigh Resistance Test\n");
	ret += sprintf(buf+ret, "=====================================================");
#else
	TOUCH_INFO_MSG("\nBin #: 12		Name: High Resistance Test\n");
#endif

	// Set report mode
	command = 0x04;
	writeRMI(F54_Data_Base, &command, 1);

	// Force update
	command = 0x04;
	writeRMI(F54_Command_Base, &command, 1);

	do {
		if(++read_count > 10) {
			TOUCH_INFO_MSG("%s[%d], command = %d\n", __func__, __LINE__, command);
			return 0;
		}
		delayMS(1); //wait 1ms
		readRMI(F54_Command_Base, &command, 1);
	} while (command != 0x00);

	command = 0x02;
	writeRMI(F54_Command_Base, &command, 1);

	read_count = 0;
	do {
		if(++read_count > 10) {
			TOUCH_INFO_MSG("%s[%d], command = %d\n", __func__, __LINE__, command);
			return 0;
		}
		delayMS(1); //wait 1ms
		readRMI(F54_Command_Base, &command, 1);
	} while (command != 0x00);

	command = 0x00;
	writeRMI(F54_Data_LowIndex, &command, 1);
	writeRMI(F54_Data_HighIndex, &command, 1);

	// Set the GetReport bit
	command = 0x01;
	writeRMI(F54_Command_Base, &command, 1);

   // Wait until the command is completed
   	read_count = 0;
	do {
		if(++read_count > 10) {
			TOUCH_INFO_MSG("%s[%d], command = %d\n", __func__, __LINE__, command);
			return 0;
		}
		delayMS(1); //wait 1ms
		readRMI(F54_Command_Base, &command, 1);
	} while (command != 0x00);

	readRMI(F54_Data_Buffer, imageBuffer, 6);

#ifdef F54_Porting
	ret += sprintf(buf+ret, "\n Parameters: ");
#else
	TOUCH_INFO_MSG("Parameters:\t");
#endif
	for(i=0; i<3; i++) {
		resistance[i] = (short)((imageBuffer[i*2+1] << 8) | imageBuffer[i*2]);
#ifdef F54_Porting
		ret += sprintf(buf+ret, "%5d ", (resistance[i]));
#else
		TOUCH_INFO_MSG("%1.3f,\t\t", (float)(resistance[i])/1000);
#endif

#ifdef F54_Porting
		if((resistance[i] >= resistanceLimit[i][0]) && (resistance[i] <= resistanceLimit[i][1]))
			Result++;
#else
		if((resistance[i]/1000 >= resistanceLimit[i][0]) && (resistance[i]/1000 <= resistanceLimit[i][1]))
			Result++;
#endif
	}
#ifdef F54_Porting
	ret += sprintf(buf+ret, "\n\n Limits(+) : ");
#else
	TOUCH_INFO_MSG("\n");

	TOUCH_INFO_MSG("Limits:\t\t");
#endif
	for(i=0; i<3; i++) {
#ifdef F54_Porting
		ret += sprintf(buf+ret, "%5d ", resistanceLimit[i][1]);
#else
		printk("%1.3f,%1.3f\t", resistanceLimit[i][0], resistanceLimit[i][1]);
#endif
	}

	ret += sprintf(buf+ret, "\n Limits(-) : ");

	for(i=0; i<3; i++) {
#ifdef F54_Porting
		ret += sprintf(buf+ret, "%5d ", resistanceLimit[i][0]);
#else
		printk("%1.3f,%1.3f\t", resistanceLimit[i][0], resistanceLimit[i][1]);
#endif
	}

#ifdef F54_Porting
	ret += sprintf(buf+ret, "\n-----------------------------------------------------\n");
#else
	printk("\n");
#endif

	// Set the Force Cal
	command = 0x02;
	writeRMI(F54_Command_Base, &command, 1);

	read_count = 0;
	do {
		if(++read_count > 10) {
			TOUCH_INFO_MSG("%s[%d], command = %d\n", __func__, __LINE__, command);
			return 0;
		}
		delayMS(1); //wait 1ms
		readRMI(F54_Command_Base, &command, 1);
	} while (command != 0x00);

	//enable all the interrupts
	//Reset
	command= 0x01;
	writeRMI(F01_Cmd_Base, &command, 1);
	delayMS(200);
	readRMI(F01_Data_Base+1, &command, 1); //Read Interrupt status register to Interrupt line goes to high

	if(Result == 3) {
#ifdef F54_Porting
		ret += sprintf(buf+ret, "RESULT: Pass\n");
		write_log(buf);
#else
		TOUCH_INFO_MSG("Test Result: Pass\n");
#endif
		return 1; //Pass
	} else {
#ifdef F54_Porting
		ret += sprintf(buf+ret, "RESULT: Fail\n");
		write_log(buf);
#else
		TOUCH_INFO_MSG("Test Result: Fail, Result = %d\n", Result);
#endif
		return 0; //Fail
	}
}

int F54_GetHighResistance(char *buf)
{
	unsigned char imageBuffer[6];
	short resistance[3];
	int i, Result=0;
	unsigned char command;

	int resistanceLimit[3][2] = { {-1000, 450}, {-1000, 450}, {-400, 20} };	//base value * 1000
	int ret = 0;
	int waitcount;

	// Set report mode
	command = 0x04;
	writeRMI(F54_Data_Base, &command, 1);

	// Force update
	command = 0x04;
	writeRMI(F54_Command_Base, &command, 1);

	waitcount = 0;
	do {
		if(++waitcount > 500) {
			TOUCH_INFO_MSG("%s[%d], command = %d\n", __func__, __LINE__, command);
		return ret;
		}
		delayMS(1); //wait 1ms
		readRMI(F54_Command_Base, &command, 1);
	} while (command != 0x00);

	command = 0x02;
	writeRMI(F54_Command_Base, &command, 1);

	waitcount = 0;
	do {
		if(++waitcount > 500) {
			TOUCH_INFO_MSG("%s[%d], command = %d\n", __func__, __LINE__, command);
			return ret;
		}
		delayMS(1); //wait 1ms
		readRMI(F54_Command_Base, &command, 1);
	} while (command != 0x00);

	command = 0x00;
	writeRMI(F54_Data_LowIndex, &command, 1);
	writeRMI(F54_Data_HighIndex, &command, 1);

	// Set the GetReport bit
	command = 0x01;
	writeRMI(F54_Command_Base, &command, 1);

	// Wait until the command is completed
	waitcount = 0;
	do {
		if(++waitcount > 500) {
			TOUCH_INFO_MSG("%s[%d], command = %d\n", __func__, __LINE__, command);
			return ret;
		}
		delayMS(1); //wait 1ms
		readRMI(F54_Command_Base, &command, 1);
	} while (command != 0x00);

	readRMI(F54_Data_Buffer, imageBuffer, 6);
/*
	ret += sprintf(buf+ret, "\n\n=====================================================\n");
	ret += sprintf(buf+ret, "\tHigh Resistance Test\n");
	ret += sprintf(buf+ret, "=====================================================\n");
	ret += sprintf(buf+ret, "\n Parameters: ");
*/
	for(i=0; i<3; i++) {
		resistance[i] = (short)((imageBuffer[i*2+1] << 8) | imageBuffer[i*2]);
		//ret += sprintf(buf+ret, "%5d ", (resistance[i]));
		if((resistance[i] >= resistanceLimit[i][0]) && (resistance[i] <= resistanceLimit[i][1]))
			Result++;
	}
/*	ret += sprintf(buf+ret, "\n\n Limits(+) : ");

	for(i=0; i<3; i++) {
		ret += sprintf(buf+ret, "%5d ", resistanceLimit[i][1]);
	}
	ret += sprintf(buf+ret, "\n Limits(-) : ");
	for(i=0; i<3; i++) {
		ret += sprintf(buf+ret, "%5d ", resistanceLimit[i][0]);
	}
*/
	// Set the Force Cal
	command = 0x02;
	writeRMI(F54_Command_Base, &command, 1);

	waitcount = 0;
	do {
		if(++waitcount > 100) {
			TOUCH_INFO_MSG("%s[%d], command = %d\n", __func__, __LINE__, command);
			break;
		}
		delayMS(1); //wait 1ms
		readRMI(F54_Command_Base, &command, 1);
	} while (command != 0x00);

	if (Result == 3) {
		ret += sprintf(buf+ret, "RESULT: Pass\n\n");
	} else {
		ret += sprintf(buf+ret, "RESULT: Fail\n\n");
	}

	//enable all the interrupts
	//	SetPage(0x00);
	//Reset
	command= 0x01;
	writeRMI(F01_Cmd_Base, &command, 1);
	delayMS(200);
	readRMI(F01_Data_Base+1, &command, 1); //Read Interrupt status register to Interrupt line goes to high

	return ret;
}
#endif

