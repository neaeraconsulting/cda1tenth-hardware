#pragma once

/*
 * Shared identity for the MAP and SPAT MessageFrames. Intersection IDs 1-255
 * are reserved for testing. Increment both MAP revisions whenever the lane
 * geometry or signal-group mapping changes.
 */
#define J2735_INTERSECTION_ID 1
#define J2735_INTERSECTION_REVISION 1
#define J2735_MAP_ISSUE_REVISION 1

/*
 * J2735 Position3D uses 1e-7 degree units. These are the standard
 * unavailable values; replace them with the surveyed table origin before
 * using GNSS/map matching.
 */
#define J2735_REFERENCE_LATITUDE 900000001L
#define J2735_REFERENCE_LONGITUDE 1800000001L

/*
 * Physical 1/10-scale geometry in centimeters. The defaults model a 35 cm
 * lane, a stop bar 50 cm from the intersection center, and a 2 m approach.
 */
#define J2735_LANE_WIDTH_CM 35
#define J2735_HALF_LANE_OFFSET_CM 18
#define J2735_STOP_BAR_OFFSET_CM 50
#define J2735_APPROACH_EXTENSION_CM 150
