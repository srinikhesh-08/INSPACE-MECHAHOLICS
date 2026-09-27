/*
 * blinker.h
 *
 *  Created on: Sep 25, 2026
 *      Author: Don Chacko
 */

#ifndef INC_BLINKER_H_
#define INC_BLINKER_H_

/*
 * On my chip the labels are off by 1.
 */
void heartbeat_Init(void);
void heartbeat_Toggle(void);

#endif /* INC_BLINKER_H_ */
