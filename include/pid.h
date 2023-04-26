#include "vex.h"

extern double target;
extern double ttarget;
extern bool turning;
extern bool straighting;
extern bool atTarget;
extern bool atAngle;
extern double pidLim;
extern bool pid_enable;
extern double derivative;
extern double error;
extern double tderivative;
extern double terror;
extern bool avging;

extern bool fwenable, shooting;
extern double fwpi, fwpp;
extern double fkp,fki,fkd;
extern double fintegral,fderivative;
extern double ferror1,fprevError,ftarget,foutput,finput;

int pid();
int flywheel_pid();
void resetPID();