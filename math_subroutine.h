// (07-26)
#ifndef MATH_SUBROUTINE
#define MATH_SUBROUTINE

#include "math.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define sind(x) (sin(fmod((x),360) * M_PI / 180))
#define cosd(x) (cos(fmod((x),360) * M_PI / 180))

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float abs_f (float value);                                                  // Absolute Value for Float Type
float adjust_angle_range (float a_angle_radian);
//double limit_angle_to_range ( double angle );								// (07-15) Limit Angle to Desired Range ( (-pi, pi] for us )
float get_abs_angle_diff ( float angle_1, float angle_2 );		// (07-15) Get Abs. Angle Diff. Within Range [0, pi]
double calc_dis_from_point_to_line ( int point[2], int line[4] );		// (08-14) Get dis from a point to a line segment


#endif
