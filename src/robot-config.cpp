#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor L1 = motor(PORT13, ratio6_1, true);
motor L2 = motor(PORT14, ratio6_1, true);
motor L3 = motor(PORT15, ratio6_1, false);
motor R3 = motor(PORT18, ratio6_1, true);
motor R2 = motor(PORT19, ratio6_1, false);
motor R1 = motor(PORT20, ratio6_1, false);
motor Shooter = motor(PORT2, ratio6_1, false);
motor Intake = motor(PORT1, ratio6_1, true);
encoder lateralEnc = encoder(Brain.ThreeWirePort.G);
encoder straightEnc = encoder(Brain.ThreeWirePort.H);
digital_out loader = digital_out(Brain.ThreeWirePort.A);
digital_out blocker = digital_out(Brain.ThreeWirePort.B);
digital_out launcher1 = digital_out(Brain.ThreeWirePort.D);
digital_out launcher2 = digital_out(Brain.ThreeWirePort.C);
digital_out dinglebopper = digital_out(Brain.ThreeWirePort.C);

signature red_goal =
    signature(1, -3441, -2785, -3113, 8975, 10355, 9665, 2.5, 0);
signature blue_goal =
    signature(1, -3441, -2785, -3113, 8975, 10355, 9665, 2.5, 0);
signature Vision5__SIG_4 = signature(4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vision goalTracker =
    vision(PORT5, 50, red_goal, blue_goal, Vision5__SIG_4,
           Vision5__SIG_4, Vision5__SIG_4, Vision5__SIG_4, Vision5__SIG_4);

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