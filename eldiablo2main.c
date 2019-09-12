#include "robot-config.h"

vex::rotationUnits rotations = vex::rotationUnits::rev;
int main() {
    int howMany = 270;
    Motor1.startRotateFor(howMany, rotations);
}
