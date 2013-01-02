/*
 * main.h 
 *
 * Description: This file is an interface to main.c
 *
 * Author: Varun Mahajan <m.varun@samsung.com>
 */

#ifndef __MAIN_H__
#define __MAIN_H__

extern int P_waitq_wkup_proc(void);

extern void P_enable_int(void);
extern void P_disable_int(void);

#endif

