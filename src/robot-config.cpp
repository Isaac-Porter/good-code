#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor L1 = motor(PORT11, ratio6_1, true);
motor L2 = motor(PORT12, ratio6_1, true);
motor L3 = motor(PORT13, ratio6_1, true);
motor R1 = motor(PORT16, ratio6_1, false);
motor R2 = motor(PORT17, ratio6_1, false);
motor R3 = motor(PORT18, ratio6_1, false);
motor Shooter = motor(PORT2, ratio6_1, true);
motor Intake = motor(PORT1, ratio36_1, true);
encoder lateralEnc = encoder(Brain.ThreeWirePort.C);
encoder straightEnc = encoder(Brain.ThreeWirePort.E);
digital_out loader = digital_out(Brain.ThreeWirePort.H);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}