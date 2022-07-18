/*
 * Buttons.h
 *
 *  Created on: Oct 2, 2021
 *      Author: cast
 */

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_
#include <stdlib.h>
#include <stdbool.h>
typedef struct
{
void (*calback)(void);
bool (*checkStateFunction)(void);
int delay;
int KeyDelay;
bool lock;
}ButtunType;
int ButtonCount=0;
ButtunType *buttons=NULL;
//===============================================================================================
void RegisterButton(bool (*checkStateFunction)(void),void (*calbackFunction)(void),int keyDelay)
{
if(ButtonCount==0){
	buttons = (ButtunType *)malloc(sizeof( ButtunType));
}
else{
	buttons = (ButtunType *)realloc(buttons,(ButtonCount+1)*sizeof( ButtunType));
}
	buttons[ButtonCount].calback=calbackFunction;
	buttons[ButtonCount].checkStateFunction=checkStateFunction;
	buttons[ButtonCount].KeyDelay=keyDelay;
	ButtonCount++;


}
//===============================================================================================
void CheckButtons()
{
for(int i=0;i<ButtonCount;i++){
if(buttons[i].checkStateFunction()==0 )
{

	if(!buttons[i].lock)
	{
		buttons[i].calback();
		buttons[i].lock=true;
		buttons[i].delay=0;
	}
	else{

		if(buttons[i].KeyDelay>=0 && buttons[i].delay++ >buttons[i].KeyDelay)
		{
			buttons[i].lock=false;

		}


	}
}
else
{
	buttons[i].lock=false;
	buttons[i].delay=0;
}
}
}
//===============================================================================================
#endif /* INC_BUTTONS_H_ */
