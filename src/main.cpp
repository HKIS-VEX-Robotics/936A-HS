#include <bits/stdc++.h>
#include <vex.h>
#include <autonomous.hpp>
#include <bot.hpp>
#include <config.hpp>
#include <pathing.hpp>

int main() {
    vexDelay(1); // DO NOT REMOVE!!! IMPORTANT FOR ODOMETRY TO ZERO
    printf("hi\n");

    #if MODE != 2
    // autonomous_task();
    control_task();
    #else
    vexDelay(1); // DO NOT REMOVE!!! IMPORTANT FOR ODOMETRY TO ZERO
    bool is_first_press = 1;
    bool left_pressed = false;
    bool right_pressed = false;

    while(is_red_side == -1 || is_far_side == -1) {
        // reset condition (both buttons)
        if (left_switch.pressing()) {
            // Wait a short time to see if the other button is also pressed
            vexDelay(50);  // 50ms window to detect second button
            if (right_switch.pressing()) {
                //reset
                is_red_side = -1;
                is_far_side = -1; 
                is_first_press = 1;
                vexDelay(300); // debounce
                continue;
            }
        }
        else if (right_switch.pressing()) {
            // short time to see if the other button is also pressed
            vexDelay(50);  // 50ms window to detect second button
            if (left_switch.pressing()) {
                //reset
                is_red_side = -1;
                is_far_side = -1; 
                is_first_press = 1;
                vexDelay(300); // debounce
                continue;
            }
        }

        if (is_first_press) {
            if (left_switch.pressing()) {
                is_red_side = false;
                is_first_press = false;
                vexDelay(300); // debounce
            }
            if (right_switch.pressing()) {
                is_red_side = true;
                is_first_press = false;
                vexDelay(300); // debounce
            }
        }

        if (!is_first_press) {
            if (left_switch.pressing()) {
                is_far_side = false;
                vexDelay(300); // debounce
            }
            if (right_switch.pressing()) {
                is_far_side = true;
                vexDelay(300); // debounce
            }
        }
        
        vexDelay(10);    
    }

    assert(is_red_side != -1);
    assert(is_far_side != -1);
    auton_id = (is_red_side << 1) | is_far_side;
    printf("red side = %d, Far side = %d\n", is_red_side, is_far_side);
    printf("auton_id = %d\n", auton_id);
    vex::competition competition;
    // competition.autonomous(autonomous_task);
    competition.drivercontrol(control_task);
    #endif

    while(true) vexDelay(1000); // DO NOT REMOVE!!! IMPORTANT TO KEEP THREADS ALIVE
}

void autonomous_task() {
    //base.follow_path({0, 60, 30, 0}, {0, 30, 60, 30}, 0.5, 0.15);
    /*
    (0, 0)
    (100, 0)
    (100, 100)
    */
    base.follow_path({0, 50, 50}, {0, 0, 50}, 0.5, 0.35);
    //vexDelay(200 MSEC);
    //base.turn_to(90 DEG, 2.5 DEG);
    //vexDelay(2 SEC);
    //base.forward(35, 1.0, 0.1);
    //base.turn_to(450 DEG, 2.5 DEG, 0.25);
}

double linear_throttling(double linear_input) {
    double output;
    int sign = linear_input / fabs(linear_input);
    linear_input = fabs(linear_input);
    if (linear_input <= 65) {
        output = pow(1.05, linear_input); 
    }
    else {
        output = linear_input;
    }
    return output * sign; 
}

void control_task() {
    double linear_speed, turn_speed, left_speed, right_speed;
    bool last_pneumatic = false, pneumatic_engaged = false;
    double turn_sens = 0.5;
    while (true) {
        linear_speed = linear_throttling(controller.Axis3.position());
        turn_speed = linear_throttling(controller.Axis1.position());

        left_speed = linear_speed + turn_speed;
        right_speed = linear_speed - turn_speed;
        if (fabs(left_speed) > 0.5) {
            left.spin(vex::fwd, left_speed, vex::pct);
        } else {
            left.stop(brakeMode);
        }
        if (fabs(right_speed) > 0.5) {
            right.spin(vex::fwd, right_speed, vex::pct);
        } else {
            right.stop(brakeMode);
        }
        if (intakeKeybind.pressing()) {
            intake_hook.spin(vex::fwd, 100, vex::pct);
            intake_roller.spin(vex::fwd, 100, vex::pct);
        } else if (outtakeKeybind.pressing()) {
            intake_hook.spin(vex::reverse, 100, vex::pct);
            intake_roller.spin(vex::reverse, 100, vex::pct);
        } else {
            intake_hook.stop(brakeMode);
            intake_roller.stop(brakeMode);
        }
        if (pneumaticKeybind.pressing() != last_pneumatic and last_pneumatic == false) {
            if (pneumatic_engaged) {
                mobile_goal.close();
            } else {
                mobile_goal.open();
            }
            pneumatic_engaged = not pneumatic_engaged;
        }
        last_pneumatic = pneumaticKeybind.pressing();
        vex::this_thread::sleep_for(10);
    }
}