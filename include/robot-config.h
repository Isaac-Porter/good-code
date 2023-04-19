using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor L1;
extern motor L2;
extern motor L3;
extern motor R1;
extern motor R2;
extern motor R3;
extern motor Shooter;
extern motor Intake;
extern digital_out loader;
extern digital_out blocker;
extern digital_out topExpansion;
extern digital_out lowExpansion1;
extern digital_out lowExpansion2;
extern digital_out dinglebopper;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );