// -----------------------------------------------------------------------------
// File:    tracking.h
// Authors: Tyler Thierolf, Timothy Miller, Garrett Smith
// Created: 10-26-2010
//
// Routines for tracking colors.
// -----------------------------------------------------------------------------

#ifndef _UAV_TRACKING__H_
#define _UAV_TRACKING__H_

#include "colordetect.h"
#include "utility.h"

// TODO: describe me
int tracking_init(client_info_t *client);

// TODO: describe me
void tracking_shutdown(void);

// TODO: describe me
void tracking_enable(int enabled);

// TODO: describe me
void tracking_set_color(track_color_t *color);

// TODO: describe me
track_color_t tracking_get_color(void);

// TODO: describe me
void tracking_set_fps(unsigned int fps);

// TODO: describe me
unsigned int tracking_get_fps(void);

// TODO: describe me
int tracking_read_state(track_coords_t *coords, access_mode_t mode);

#endif // _UAV_TRACKING__H_

