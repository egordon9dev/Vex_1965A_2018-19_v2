#include "main.h"
#include "setup.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Initializing");
    pros::lcd::set_text(2, "Stand Back");
    pros::lcd::set_text(3, "Don't move");
    pros::lcd::set_text(4, "I'm prepin stuff");
    pros::lcd::set_text(5, "so we win");
    pros::lcd::set_text(6, "...");
    pros::lcd::set_text(7, "...");
    printf("initializing...\n");
    setup();
    printf("Ready\n");
    for (int i = 1; i < 8; i++) { pros::lcd::set_text(i, ""); }
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}