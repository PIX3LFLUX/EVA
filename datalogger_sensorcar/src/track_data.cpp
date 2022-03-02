#include "track_data.h"

/* current track geometry data */
DRAM_ATTR uint8_t number_track_pieces = 8;  /* total number of track pieces that the track is made out of */

/* array of integers that holds values representing track pieces. The 0th element is the starting position, so it's usually the straight. Other possible geometries include the inner and outer left and right curves. */
DRAM_ATTR uint8_t track_geometry[50] = { 0 };

/* holds double values of the total track length at every track piece length. For example: the element at index 0 is always 0, the element at index 1 has the length of the 1st track piece, the element at index 2 has the lentgh of the first two elements, etc. It is used as a reference for position along the track. */
DRAM_ATTR double_t track_checkpoint_lengths[50] = { 0 };

/* calculate track length at each checkpoint */
IRAM_ATTR void calculate_track_checkpoint_lengths(uint8_t number_track_pieces)
{
    double_t tracklength_sum = 0.0;
    for(uint8_t ii = 0; ii < number_track_pieces - 1; ii++)
    {
        tracklength_sum += TRACKPIECE_LENGTH[track_geometry[ii]];
        track_checkpoint_lengths[ii+1] = tracklength_sum;
        #if DEBUG
        Serial.printf("track_checkpoint_lengths[%d] = %lf\n", ii+1, track_checkpoint_lengths[ii+1]);
        #endif
    }
}

/* determine track piece based on ir_left_right_time_difference and empirically gathered data */
IRAM_ATTR uint8_t determine_track_piece(signed long sensor_time_difference)
{
    long absolute_sensor_time_difference = abs(sensor_time_difference);
    bool is_positive = (sensor_time_difference > 0);

    if (absolute_sensor_time_difference < THRESHOLD_STRAIGHT)
    {
        return TRACK_STRAIGHT;
    }
    else if ((absolute_sensor_time_difference > THRESHOLD_CURVE_OUTER) && (absolute_sensor_time_difference < THRESHOLD_CURVE_INNER))
    {
        if (is_positive)  { return TRACK_CURVE_LEFT_OUTER_TRACK; }
        else              { return TRACK_CURVE_RIGHT_OUTER_TRACK; }
    }
    else if (absolute_sensor_time_difference > THRESHOLD_CURVE_INNER)
    {
        if (is_positive)  { return TRACK_CURVE_LEFT_INNER_TRACK; }
        else              { return TRACK_CURVE_RIGHT_INNER_TRACK; }
    }
    
    return 0;   /* default value if none is applicable */
}