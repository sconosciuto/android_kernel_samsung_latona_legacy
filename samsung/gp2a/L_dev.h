/*
 * L_dev.h 
 *
 * Description: This file is an interface to L_dev.c
 *
 * Author: Varun Mahajan <m.varun@samsung.com>
 */

#ifndef _L_DEV_H
#define _L_DEV_H

/*Function prototypes*/
/**************************************************************/
extern void L_dev_sync_mech_init(void); 

extern int L_dev_init(void);
extern int L_dev_exit(void);

extern int L_dev_suspend(void);
extern int L_dev_resume(void);

extern int L_dev_get_adc_val(u32 *);
extern int L_dev_polling_start( void );
extern int L_dev_polling_stop( void );
extern int L_dev_get_polling_state(void);
extern unsigned int L_dev_get_polling_interval( void );
extern void L_dev_set_polling_interval(unsigned int interval);

/**************************************************************/

#endif

