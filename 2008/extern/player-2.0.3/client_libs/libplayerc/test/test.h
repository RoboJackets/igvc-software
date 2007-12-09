/***************************************************************************
 * Desc: Test program for the Player C client
 * Author: Andrew Howard
 * Date: 23 May 2002
 # CVS: $Id: test.h,v 1.23 2006/04/11 15:40:31 veedee Exp $
 **************************************************************************/

#ifndef TEST_H
#define TEST_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "playerc.h"

// Message macros
#define TEST(msg) (1 ? printf(msg " ... "), fflush(stdout) : 0)
#define TEST1(msg, a) (1 ? printf(msg " ... ", a), fflush(stdout) : 0)
#define PASS() (1 ? printf("pass\n"), fflush(stdout) : 0)
#define FAIL() (1 ? printf("\033[41mfail\033[0m\n%s\n", playerc_error_str()), fflush(stdout) : 0)

// Basic laser test
extern int test_laser(playerc_client_t *client, int index);

// Basic test for position2d device.
extern int test_position2d(playerc_client_t *client, int index);

// Basic test for position3d device
extern int test_position3d(playerc_client_t *client, int index);

// Basic test for log device.
extern int test_log(playerc_client_t *client, int index);

// Basic test for simulation device.
extern int test_simulation(playerc_client_t *client, int index);

// Basic test for sonar device.
extern int test_sonar(playerc_client_t *client, int index);

// Basic test for power device.
extern int test_power(playerc_client_t *client, int index);

// Basic test for map device
extern int test_map(playerc_client_t *client, int index);

// Basic test for camera device
extern int test_camera(playerc_client_t *client, int index);

// Basic blobfinder test
extern int test_blobfinder(playerc_client_t *client, int index);

// Basic test for ptz device.
extern int test_ptz(playerc_client_t *client, int index);

#if 0
// Basic test for BPS device.i
extern int test_bps(playerc_client_t *client, int index);

// Basic broadcast test
extern int test_broadcast(playerc_client_t *client, int index);

// Basic test for GPS device.i
extern int test_gps(playerc_client_t *client, int index);

// Basic localization test
extern int test_localize(playerc_client_t *client, int index);

// Basic test for the LBD (laser beacon detector) device.
// REMOVE? int test_lbd(playerc_client_t *client, int index);

// Basic localization test
int test_localize(playerc_client_t *client, int index);

// Basic test for truth device.
extern int test_truth(playerc_client_t *client, int index);

// Basic vision test.
extern int test_vision(playerc_client_t *client, int index);

// Basic comms device test
extern int test_comms(playerc_client_t *client, int index);

// Basic test for the laser beacon device.
extern int test_fiducial(playerc_client_t *client, int index);

// Basic test for wifi device.
extern int test_wifi(playerc_client_t *client, int index);
#endif

// Basic test for Graphics2D.
extern int test_graphics2d(playerc_client_t *client, int index);

// Basic test for Gripper.
extern int test_gripper(playerc_client_t *client, int index);

// Basic test for RFID.
extern int test_rfid(playerc_client_t *client, int index);

// Basic test for WSN.
extern int test_wsn(playerc_client_t *client, int index);

#endif // TEST_H
