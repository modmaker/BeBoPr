#ifndef _MENDEL_H
#define _MENDEL_H

#include <pthread.h>

extern int mendel_thread_create( const char* name, pthread_t* restrict thread, const pthread_attr_t* restrict attr,
				 void* (*worker_thread)( void*), void* restrict arg);
extern int mendel_sub_init( const char* name, int (*subsys)( void));

#endif
