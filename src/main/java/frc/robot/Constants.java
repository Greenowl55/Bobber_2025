// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

// You should use this. -4674
// You should use this. -4674
// You should use this. -4674

public class Constants {
    public static final String LIMELIGHT4_NAME = "limelight-four";
    // Elevator motors
    public static final int ELEVATOR_LEADER = 11;
    public static final int ELEVATOR_FOLLOWER = 12;

    // Elevator positions
    public static final double ELEVATOR_BOTTOM = 0.0;
    public static final double ELEVATOR_INTAKE = 6.5;
    public static final double ELEVATOR_L2 = 16.5;
    public static final double ELEVATOR_L3 = 30.5;
    public static final double ELEVATOR_L4 = 61.0;

    // Fishhook Motors
    public static final int TILT_MOTOR_ID = 8;
    public static final int ALGAE_MOTOR_ID = 9;
    public static final int CORAL_MOTOR_ID = 10;
    public static final int CORAL_SENSOR = 0;

    // Game piece speeds
    public static final double ALGAE_HOLD = -0.2;
    public static final double ALGAE_IN = -0.45;
    public static final double ALGAE_OUT = 0.3;
    public static final double CORAL_INTAKE = -0.075;
    public static final double CORAL_SLOW = -0.1;
    public static final double CORAL_FAST = -0.25;

    // fish hook setpoints
    public static final double FISHHOOK_IN = 1;
    public static final double FISHHOOK_L4 = 4.75;
    public static final double FISHHOOK_ALGAE = 6;
    public static final double FISHHOOK_GROUND = 9;

    // Climber Motor
    public static final int CLIMBER_MOTOR_ID = 14;

    // Climber setpoints
    public static final int CLIMBER_OUT =  570;
    public static final int CLIMBER_IN = 0;
    public static final double CLIMBER_OUT_MANUAL = -1;
    public static final double CLIMBER_IN_MANUAL = 1;
    // TODO: Set these values

    // CoDriver Buttons
    public static final int CODRIVER_1 = 1;
    public static final int CODRIVER_2 = 2;
    public static final int CODRIVER_3 = 3;
    public static final int CODRIVER_4 = 4;
    public static final int CODRIVER_5 = 5;
    public static final int CODRIVER_6 = 6;
    public static final int CODRIVER_7 = 7;
    public static final int CODRIVER_8 = 8;
    public static final int CODRIVER_9 = 9;
    public static final int CODRIVER_10 = 10;
    public static final int CODRIVER_11 = 11;

    // Auto things
    public static final double MOTOR_POSITION_EPSILON = 0.05;
    public static final int AUTO_TIMEOUT = 1;

    // Reef April Tag array
    public static final int[] REEF_TAGS = {
            2,/*  2 This is not a field tag */ 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22
    };

    //LED things
    public static final int LED_PORT = 0;
    public static final int LED_PORT2 = 2;
    public static final int MAST_LENGTH = 35;
    public static final int BAR_LENGTH2 = 32;
    public static final int LED_LENGTH = MAST_LENGTH + BAR_LENGTH2;

    // AdvantageKit and AdvantageScope things
    public static final boolean isCompetitionReady = true;	// Set to false for testing and true for comps
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

      public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
  }

}
