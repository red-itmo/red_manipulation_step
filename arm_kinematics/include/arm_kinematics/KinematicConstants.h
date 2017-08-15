#ifndef KINEMATIC_CONSTANTS
#define KINEMATIC_CONSTANTS

// Youbot manipulator parameters from URDF file
const double d0x = 0.024;
const double d0z = 0.096;
const double d1x = 0.033;
const double d1z = 0.019;
const double d2 = 0.155;
const double d3 = 0.135;
const double d4 = 0.13;
const double griperLength = 0.105;

const double jointMinAngles[5] = {-2.8905336, -1.1243948, -2.4783693, -1.7668386, -2.7181635};
const double jointMaxAngles[5] = {2.9395372, 1.483526, 2.5324727, 1.6402375, 2.8128075};
const double jointOffsets[5] = {2.9496064359, 1.1344640138, -2.5481807079, 1.7889624833, 2.9234264971};

#endif