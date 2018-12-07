/*
 * AT_Commands.c
 *
 *  Created on: 07 Dec 2018
 *      Author: HermiduPlessis
 */

#include "AT_Commands.h"

void AT_Command_Write(uint8_t *AT_Command,uint8_t AT_Size, uint8_t *Setting, uint8_t Setting_Size)
{
	uint8_t offset = 7;
	uint8_t send_Size = AT_size+Setting_Size+offset;

	uint8_t send_Array[send_Size] = {0};

	send_Array[0] = 'A';
	send_Array[1] = 'T';


	for(uint8_t i = 0; i < AT_Size+2; i++)
	{
		send_Array[i+2] = AT_Command[i];
	}

	send_Array[AT_Size+offset-5] = '/';
	send_Array[AT_Size+offset-4] = 'r';
	send_Array[AT_Size+offset-3] = '/';
	send_Array[AT_Size+offset-2] = 'n';
	send_Array[AT_Size+offset-1] = '=';

	for(uint8_t i = 0; i < Setting_Size; i++)
	{
		send_Array[AT_Size+i+offset] = Setting[i];
	}

	if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)send_Array, send_Size)== HAL_OK)
	{

	}
	else
	{

	}

}

void AT_Command_Read(uint8_t *AT_Command,uint8_t AT_Size)
{
	uint8_t offset = 3;
	uint8_t send_Size = AT_size;

	uint8_t send_Array[send_Size] = {0};

	send_Array[0] = 'A';
	send_Array[1] = 'T';


	for(uint8_t i = 0; i < AT_Size+2; i++)
	{
		send_Array[i+2] = AT_Command[i];
	}

	send_Array[AT_Size+offset-5] = '/';
	send_Array[AT_Size+offset-4] = 'r';
	send_Array[AT_Size+offset-3] = '/';
	send_Array[AT_Size+offset-2] = 'n';
	send_Array[AT_Size+offset-1] = '?';


	if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)send_Array, send_Size)== HAL_OK)
	{

	}
	else
	{

	}
}

void AT_Response(uint8_t *receive)
{


}


