#ifndef __Queue_H
#define __Queue_H

#include<stdio.h>
#include<stdlib.h>

typedef int Status ;
typedef struct QNode
{
    unsigned char data[1024] ;
    struct QNode *next ;
} QNode ;

typedef struct LinkQueue
{
    QNode *front , *rear ;
} LinkQueue ;

void InitLinkQueue( LinkQueue *Q );
void TraverLinkQueue( LinkQueue Q ) ;			//遍历队列
Status EnLinkQueue( LinkQueue *Q ,unsigned char *val) ;	//进队
Status DeLinkQueue( LinkQueue *Q ) ;	        //出队
Status ClearLinkQueue( LinkQueue *Q ) ;			//清空队列
Status DesLinkQueue( LinkQueue *Q ) ;			//销毁队列
Status GetHead( LinkQueue Q  ,unsigned char *val) ;		//取头结点
int GetLength( LinkQueue Q ) ;					//求队长

#endif

