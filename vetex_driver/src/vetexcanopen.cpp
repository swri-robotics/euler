/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

!!!!!!!!!!!!!!!!!!!!!!!!!!
This file was originally for the CANOpenShell program included with the
CANFestival examples. It has been heavily (HEAVILY) modified to be used
as this vetexcanopen library/program.
*/

#include <unistd.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

#include "canfestival.h"
#include "vetexcanopen.h"
#include "vetexcanopenSlaveOD.h"

//****************************************************************************
// DEFINES
//****************************************************************************
#define CAN_DRIVER "libcanfestival_can_socket.so"
#define JOYSTICK_NODE_ID 0x0a

//****************************************************************************
// LOCAL VARIABLES
//****************************************************************************
char BoardBusName[31] = "can0";
char BoardBaudRate[5] = "250K";
s_BOARD Board = {BoardBusName, BoardBaudRate};
CO_Data* CANOpenShellOD_Data;
CAN_PORT can_port;
Message vel_msg;

//****************************************************************************
// CALLBACK FUNCTIONS
//****************************************************************************
void CANOpenShellOD_initialisation(CO_Data* d)
{
	printf("Node_initialisation\n");
}

void CANOpenShellOD_preOperational(CO_Data* d)
{
	printf("Node_preOperational\n");
}

void CANOpenShellOD_operational(CO_Data* d)
{
	printf("Node_operational\n");
}

void CANOpenShellOD_stopped(CO_Data* d)
{
	printf("Node_stopped\n");
}

void CANOpenShellOD_post_SlaveBootup(CO_Data* d, UNS8 nodeid)
{
	printf("Slave %x boot up\n", nodeid);
}

void CANOpenShellOD_post_TPDO(CO_Data* d)
{
	//printf("Master_post_TPDO\n");
}

void CANOpenShellOD_post_sync(CO_Data* d)
{
	// Every time a SYNC comes around from the master, send out the PDO message
	// which tells the master what the current "joystick" position is. We're
	// transmitting x,y,yaw,enable as a single message.

	int ret = canSend(can_port, &vel_msg);
	if (ret != 0) {
		printf("Ret from canSend: %d\n" , ret);
	}
	//printf("Master_post_sync\n");
}

//****************************************************************************
// Initialization/Destruction
//****************************************************************************
void Init(CO_Data* d, UNS32 id)
{
	/* Init node state*/
	setState(CANOpenShellOD_Data, Initialisation);
}

void Exit(CO_Data* d, UNS32 id)
{

}

int NodeInit(int NodeID)
{
 	CANOpenShellOD_Data = &CANOpenShellSlaveOD_Data;

	/* Load can library */
	LoadCanDriver(CAN_DRIVER);

	/* Define callback functions */
	CANOpenShellOD_Data->initialisation = CANOpenShellOD_initialisation;
	CANOpenShellOD_Data->preOperational = CANOpenShellOD_preOperational;
	CANOpenShellOD_Data->operational = CANOpenShellOD_operational;
	CANOpenShellOD_Data->stopped = CANOpenShellOD_stopped;
	CANOpenShellOD_Data->post_sync = CANOpenShellOD_post_sync;
	CANOpenShellOD_Data->post_TPDO = CANOpenShellOD_post_TPDO;
	CANOpenShellOD_Data->post_SlaveBootup=CANOpenShellOD_post_SlaveBootup;

	/* Open the Peak CANOpen device */
	can_port = canOpen(&Board,CANOpenShellOD_Data);
	if (!can_port) return -1;
	/* Defining the node Id */
	setNodeId(CANOpenShellOD_Data, NodeID);
	/* Start Timer thread */
	StartTimerLoop(&Init);
	return 0;
}

void vetex_initialize(void)
{
	/* Init stack timer */
	TimerInit();
	int ret = NodeInit(JOYSTICK_NODE_ID);

	// Setup the velocity message
	// NOTE - This is being done down at the raw CAN level, not at a real
	// CANOpen style of level. This is for expediency, because I'm copying the
	// packet from captures, and it works.
	// This packet will get sent out everytime that a SYNC is received over
	// the CAN bus. This packet gets reconfigured based on functions that
	// are called to set the velocity.
	vel_msg.cob_id = 0x18a;
	vel_msg.rtr = 0;
	vel_msg.len = 8;
	vel_msg.data[0] = 0xf4;
	vel_msg.data[1] = 0x01;
	vel_msg.data[2] = 0xf4;
	vel_msg.data[3] = 0x01;
	vel_msg.data[4] = 0xf4;
	vel_msg.data[5] = 0x01;
	vel_msg.data[6] = 0x40;
	vel_msg.data[7] = 0x00;
}

void vetex_terminate(void)
{
	// Stop timer thread
	StopTimerLoop(&Exit);

	/* Close CAN board */
	canClose(CANOpenShellOD_Data);

	TimerCleanup();
}

//****************************************************************************
// TESTING
//****************************************************************************
void Dance(void)
{
	time_t t = time(NULL);
	switch (t % 10) {
		case 0:
			vetex_disable_movement();
			vetex_set_all_percentages(0, 0, 0);
			break;
		case 1:
			vetex_enable_movement();
			break;
		case 2:
			vetex_set_all_percentages(25, 0, 0);
			break;
		case 3:
			vetex_set_all_percentages(0, 25, 0);
			break;
		case 4:
			vetex_set_all_percentages(-25, 0, 0);
			break;
		case 5:
			vetex_set_all_percentages(0, -25, 0);
			break;
		case 6:
			vetex_set_all_percentages(0, 0, 25);
			break;
		case 7:
			vetex_set_all_percentages(0, 0, -25);
			break;
		case 8:
			vetex_set_all_percentages(25, 25, 0);
			break;
		case 9:
			vetex_set_all_percentages(-25, -25, 0);
			break;
	}
}

//****************************************************************************
// PARAMETER SETTING CODE
//
// set_x/y/z_position functions are directly pretending to be the analog
// joystick. They take a value from 100 to 900, with 500 being the center
// value.
// X +/- is right/left, Y +/- is forward/backward, Z +/- is rotate CW/CCW
//
// set_x/y/z_percentage is similar, but uses percentages instead of ticks,
// in the range of -100 to 100% (with 0 being no movement)
//****************************************************************************
void vetex_set_x_position(uint16_t ticks)
{
	if (ticks < 100 || ticks > 900) {
		return;
	}
	vel_msg.data[0] = ticks & 0xFF;
	vel_msg.data[1] = (ticks >> 8) & 0xFF;
}

void vetex_set_y_position(uint16_t ticks)
{
	if (ticks < 100 || ticks > 900) {
		return;
	}
	vel_msg.data[2] = ticks & 0xFF;
	vel_msg.data[3] = (ticks >> 8) & 0xFF;
}

void vetex_set_z_position(uint16_t ticks)
{
	if (ticks < 100 || ticks > 900) {
		return;
	}
	vel_msg.data[4] = ticks & 0xFF;
	vel_msg.data[5] = (ticks >> 8) & 0xFF;
}

void vetex_enable_movement()
{
	vel_msg.data[6] = 0x41;
}

void vetex_disable_movement()
{
	vel_msg.data[6] = 0x40;
}

void vetex_set_x_percentage(int8_t percent)
{
	if (percent < -100 || percent > 100) {
		return;
	}
	uint16_t ticks = ((int16_t)percent * 4) + 500;
	vetex_set_x_position(ticks);
}

void vetex_set_y_percentage(int8_t percent)
{
	if (percent < -100 || percent > 100) {
		return;
	}
	uint16_t ticks = ((int16_t)percent * 4) + 500;
	vetex_set_y_position(ticks);
}

void vetex_set_z_percentage(int8_t percent)
{
	if (percent < -100 || percent > 100) {
		return;
	}
	uint16_t ticks = ((int16_t)percent * 4) + 500;
	vetex_set_z_position(ticks);
}

void vetex_set_all_percentages(int8_t x, int8_t y, int8_t z)
{
	vetex_set_x_percentage(x);
	vetex_set_y_percentage(y);
	vetex_set_z_percentage(z);
}
//
///****************************************************************************/
///***************************  MAIN  *****************************************/
///****************************************************************************/
//
//int main(int argc, char** argv)
//{
//	vetex_initialize();
//
//	while (true)
//	{
//		Dance();
//		usleep(500);
//	}
//
//	vetex_terminate();
//
//	return 0;
//}
