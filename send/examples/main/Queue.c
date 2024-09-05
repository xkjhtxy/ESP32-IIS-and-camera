#include "Queue.h"
#include "driver/spi_slave.h"
extern spi_slave_transaction_t t;
void InitLinkQueue( LinkQueue *Q )
{
    Q->front = ( QNode * )malloc( sizeof( QNode ) ) ;
    Q->rear = Q->front;
    Q->rear->next = NULL ;
}

void TraverLinkQueue( LinkQueue Q )
{
    QNode *p = Q.front->next ;
    while( p )
    {
        printf( "%s " , p->data ) ;
        p = p->next ;
    }
    printf( "\n" ) ;
}

Status EnLinkQueue( LinkQueue *Q,unsigned char *val)
{
    QNode *p ;
    int i;
    p = ( QNode * )malloc( sizeof( QNode ) ) ;

    //p->data = val ;
    for(i=0;i<1024;i++){
		p->data[i] = val[i] ;
	}
    //printf( "进队数据:%s " , p->data ) ;
	//t.rx_buffer = p->data;
    p->next = NULL ;

    Q->rear->next = p ;
    Q->rear  = p ;
    return 1 ;
}

Status DeLinkQueue( LinkQueue *Q )
{
    //int i;
    if( Q->front == Q->rear )
        return 0 ;
    else
    {
        QNode *p = Q->front->next ;
        // for(i=0;i<1024;i++){
    	// 	(val[i]) = p->data[i] ;
		// }

        //*val = p->data ;

        if( Q->front->next == Q->rear )
        {
            Q->rear = Q->front ;
            Q->front->next = NULL ;
        }
        else
            Q->front->next = p->next ;
        free( p ) ;
    }

    return 1 ;
}

Status ClearLinkQueue( LinkQueue *Q )
{
    if( !Q )
        return 0 ;
    else
    {
        QNode *p ;
        p = Q->front->next ;
        Q->rear = Q->front ;

        while( p )
        {
            Q->front->next = p->next ;
            free( p ) ;
            p = Q->front->next ;
        }

        return 1 ;
    }
}

Status DesLinkQueue( LinkQueue *Q )
{
    if( !Q )
        return 0 ;
    else
    {
        while( Q->front )
        {
            Q->rear = Q->front->next ;
            free( Q->front ) ;
            Q->front = Q->rear ;
        }
        return 1 ;
    }
}

int GetLength( LinkQueue Q )
{
    int length = 0 ;
    QNode *p = Q.front->next ;
    while( p )
    {
        length++ ;
        p = p->next ;
    }
    return length ;
}

Status GetHead( LinkQueue Q  ,unsigned char *val)
{
    int i;
    if( Q.front == Q.rear )
        return 0 ;
    else
    {
		for(i=0;i<1024;i++){
            (val[i]) = Q.front->next->data[i] ;
		}
        //printf( "出队数据:%s " , Q.front->next->data ) ;
       // *val = Q.front->next->data ;
	   //printf("%s",Q.front->next->data);
        return 1 ;
    }
}
