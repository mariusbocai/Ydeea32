/*
 * typeDefs.h
 *
 *  Created on: Nov 19, 2019
 *      Author: bocaim
 */

#ifndef SRC_TYPEDEFS_H_
#define SRC_TYPEDEFS_H_

/*===== Acceptable return types for API's =====*/
enum {
	E_NOK = 0,	/* The function returned all the values correct and updated */
	E_OK 		/* Either the values could not be returned because of errors (communication, etc) or the values are not all up to date*/
};

enum {
	stateInit = 0,
	stateCalibration,
	stateRunning
};

#define okToLeaveState (LEAVE_STATE)

#endif /* SRC_TYPEDEFS_H_ */
