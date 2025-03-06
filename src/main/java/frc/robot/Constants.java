// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

// You should use this. -4674
// You should use this. -4674
// You should use this. -4674

public class Constants {

    // Elevator motors
    public static final int ELEVATOR_LEADER = 11;
    public static final int ELEVATOR_FOLLOWER = 12;

    // Elevator positions
    public static final double ELEVATOR_BOTTOM = 0.0;
    public static final double ELEVATOR_INTAKE = 8.0;
    public static final double ELEVATOR_L2 = 18;
    public static final double ELEVATOR_L3 = 32.0;
    public static final double ELEVATOR_L4 = 63.0;

    // Fishhook Motors
    public static final int TILT_MOTOR_ID = 8;
    public static final int ALGAE_MOTOR_ID = 9;
    public static final int CORAL_MOTOR_ID = 10;
    public static final int CORAL_SENSOR = 0;

    // Game piece speeds
    public static final double ALGAE_IN = -0.5;
    public static final double ALGAE_OUT = 0.5;
    public static final double CORAL_SLOW = -0.2;
    public static final double CORAL_FAST = -0.35;

    // fish hook setpoints
    public static final double FISHHOOK_IN = 0;
    public static final double FISHHOOK_L4 = 3.5;
    public static final double FISHHOOK_GROUND = 9;

    // Climber Motor
    public static final int CLIMBER_MOTOR_ID = 14;

    // Climber setpoints
    // TODO: Set these values

    // CoDriver Buttons
    public static final int CODRIVER_TRIGGER = 1;
    public static final int CODRIVER_BOTTOM_FACE = 2;
    public static final int CODRIVER_LEFT_FACE = 3;
    public static final int CODRIVER_RIGHT_FACE = 4;
    public static final int CODRIVER_5 = 5;
    public static final int CODRIVER_6 = 6;
    public static final int CODRIVER_7 = 7;
    public static final int CODRIVER_8 = 8;
    public static final int CODRIVER_9 = 9;
    public static final int CODRIVER_10 = 10;

    // Auto things
    public static final double MOTOR_POSITION_EPSILON = 0.05;
    public static final int AUTO_TIMEOUT = 1;

    // Reef April Tag array
    public static final int[] REEF_TAGS = {
            6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22
    };

}
