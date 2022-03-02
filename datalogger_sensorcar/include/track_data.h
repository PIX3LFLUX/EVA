#include "globals.h"

#define TRACK_STRAIGHT                  0
#define TRACK_CURVE_LEFT_INNER_TRACK    1
#define TRACK_CURVE_LEFT_OUTER_TRACK    2
#define TRACK_CURVE_RIGHT_INNER_TRACK   3
#define TRACK_CURVE_RIGHT_OUTER_TRACK   4

/* Absolute time thresholds. Based on the time difference it takes the IR sensors to trigger, determine the track piece. Times in us, set speed of 40. Test track: simplest zero with right curves. Strongly dependent on actual car speed, which actually varies based on the geometry. */
#define THRESHOLD_STRAIGHT              2500
#define THRESHOLD_CURVE_INNER           18000   /* typical values: 16000...22000 */
#define THRESHOLD_CURVE_OUTER           10000   /* typical values: 13000...17000 */

/* array that contains track piece length in meters. The index number corresponds to the track pieces as defined above. For example, TRACKPIECE_LENGTH[0] has the length for the 'TRACK_STRAIGHT' pieces, because it is defined as '#define TRACK_STRAIGHT 0'*/
const double_t TRACKPIECE_LENGTH[] = { 345e-3, 259.181393921e-3, 362.85395149e-3, 259.181393921e-3, 362.85395149e-3 }; 

/* contains the target speed one can drive on a trackpiece without derailing or losing speed from sliding in a corner. Data obtained empirically. */
const uint8_t TARGET_TRACKPIECE_SPEED_DIGITAL[] = { 62, 44, 44, 44, 44 }; 

/* All possible individual target vdigis. All other values just result in the same result as one of these values due to quantization by the carrera CU. */
const uint8_t AVAILABLE_VDIGI[]             = { 0, 20, 26, 32, 38, 44, 50, 56, 62, 68, 74, 80, 86, 92, 98 };

/* current track geometry data */
extern DRAM_ATTR uint8_t number_track_pieces;
extern DRAM_ATTR uint8_t track_position_index;
extern DRAM_ATTR uint8_t track_geometry[50];
extern DRAM_ATTR double_t track_checkpoint_lengths[50];

IRAM_ATTR void      calculate_track_checkpoint_lengths(uint8_t number_track_pieces);
IRAM_ATTR uint8_t   determine_track_piece(signed long sensor_time_difference);