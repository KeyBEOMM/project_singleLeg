#include <iostream>
#include "core_lib/kinematics.h"

int main() {
    leg_FK legFK;

    legFK.FK_solver();

    return 0;   
}