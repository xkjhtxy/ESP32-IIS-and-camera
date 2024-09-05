#ifndef __mServer_H
#define __mServer_H

#include "mWifi.h"

void mServer_Init(void *pvParameters);
// void mClient_Init(void *pvParameters);
void Tcp_Receive_Task();
#endif

