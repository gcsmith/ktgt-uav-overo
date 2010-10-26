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

int tracking_init(client_info_t *client);

void tracking_shutdown(void);

void tracking_enable(int enabled);

void tracking_set_color(track_color_t *color);

track_color_t tracking_get_color(void);

void tracking_set_fps(unsigned int fps);

unsigned int tracking_get_fps(void);

#endif // _UAV_TRACKING__H_

