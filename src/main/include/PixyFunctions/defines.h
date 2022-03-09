// Pixy Block Filtering Settings
//#define Y_MIN_THRESHOLD                 55  // 0 is top of vision frame ?
//#define Y_MAX_THRESHOLD                 90  // 160 is bottom of vision frame ?
#define Y_MIN_THRESHOLD                 50  // 0 is top of vision frame ?
#define Y_MAX_THRESHOLD                160  // 160 is bottom of vision frame ?
#define MAX_VALID_BLOCKS                10

#define MAX_BLOCK_WIDTH_DIFFERENCE       8  // maximum difference in number of reflector width pixels to be considered to belong to a target
#define MAX_BLOCK_HEIGHT_DIFFERENCE      8  // maximum difference in number of reflector height pixels to be considered to belong to a target
#define FILTER_CENTER_X                160  // center X value in  camera frame ?

// Camera location relative to object placement position
#define PIXY_0_X_OFFSET                -85  // mm, pixy 0 camera position offset from center of robot, negative = left from center
#define PIXY_0_Y_OFFSET               -900  // mm, pixy 0 camera position offset from front of robot (object placement point), negative = backward from front

// Target Distance Calculation Settings
#define DISTANCE_CALIBRATION_GAIN        7597.329
#define DISTANCE_CALIBRATION_EXPONENT  -0.9984968

// Robot Angle Calculation Settings
#define IMAGE_CENTER_X                 160  // X value at center of image (1/2 VGA resolution?)
#define DISTANCE_BETWEEN_TARGETS       290  // mm between the centers of the 2 target reflectors

// Target Angle Calculation Settings
#define TA_CALC_MAX_DELTA_H           0.15  // Target Angle 15 percent difference is maximum to calculate target angle from
#define TA_CALIBRATION_GAIN         2.1746  // Target Angle calibration gain
#define TA_CALIBRATION_OFFSET       -2.219  // Target Angle calibration offset

// Trajectory Calculation Settings
//#define CALC_TRAJECTORY_INTERVAL     100  // interval in millis between trajectory recalculations
#define CALC_TRAJECTORY_INTERVAL     200  // interval in millis between trajectory recalculations
//#define CALC_TRAJECTORY_INTERVAL    5000  // interval in millis between trajectory recalculations

#define BEZIER_NUMBER_OF_POINTS       20  // number of points
#define BEZIER_RESOLUTION           0.05  // 1 / number of points

#define MIN_DIST_CALC_TRAJECTORY     300  // mm (was 250)
#define AUTO_DRIVE_MIN_DISTANCE      250  // mm
#define EXP_FILT_SPEED_CONSTANT      1.0  // 0.0 to 1.0, 1.0 is no filtering

// Robot Physical Settings
#define ROBOT_TRACK_WIDTH            550  // mm
#define ROBOT_MIN_SPEED               45  // mm/s  was 50
#define ROBOT_MAX_SPEED               65  // mm/s  was 70
#define ROBOT_ACCELERATION             2  // mm/s / setpoint speed calculation cycle
#define ROBOT_DECELERATION             2  // mm/s / setpoint speed calculation cycle

#define ARM_RETRACTED_LENGTH        1500  // mm arm length in retracted position
#define ARM_Y_OFFSET_BASE           1000  // mm base length from where to compensate arm y offset
#define WRIST_BALL_Y_OFFSET          400  // mm difference in y offset between ball placement and hatch placement

// Serial Data Communication Settings
#define TX_MAX_BLOCKS                 10  // maximum number of blocks to send in one $PRBLK message
#define TX_PRBLK_INTERVAL           2000  // $PRBLK transmit interval in milliseconds
#define TX_PRTAR_INTERVAL            100  // $PRTAR transmit interval in milliseconds
#define TX_PRTRA_INTERVAL           2000  // $PRTRA transmit interval in milliseconds
#define TX_PRSWS_INTERVAL            100  // $PRSWS transmit interval in milliseconds
