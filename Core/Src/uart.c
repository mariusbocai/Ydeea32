/*===== Interface to the UART =====*/

#define fact 1

void serialInit()
{

}

/* Send one character on serial, function is blocking */
static void sendSingleChar(unsigned char valToSend)
{

}

void serialSendChar(double* value, unsigned char newLine)
{
	int value_int;
	unsigned int value_uint;
	unsigned char separateChars[5],index;

	index = 0;
	if(*value < 0)
	{
		sendSingleChar(45);		/* Send a "-" */
	}
	value_int = (int)((*value));
	if(*value < 0)
	{
		value_uint = ~value_int + 1;
	}
	else
	{
		value_uint = value_int;	
	}
	do
	{
		separateChars[index++] = value_uint%10;
		value_uint /= 10;
	}
	while(value_uint > 0);
	for(;index>0;index--)
	{
		sendSingleChar((unsigned char)(separateChars[index-1]+48));
	}	
	if(newLine==1)
	{
		sendSingleChar(10); 
	}
	else
	{
		/*if this is not the end of the row, send a ","*/
		sendSingleChar(44);
	}
}
