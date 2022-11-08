// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Drive {

        public static final int LEFT_FRONT_MOTOR_PORT = 4;
        public static final int LEFT_BACK_MOTOR_PORT = 9;
        public static final int RIGHT_FRONT_MOTOR_PORT = 5;
        public static final int RIGHT_BACK_MOTOR_PORT = 1;

        public static final boolean LEFT_DRIVE_INVERTED = false;
        public static final boolean RIGHT_DRIVE_INVERTED = true;

        public enum DifferentialControlScheme {
            ARCADE,
            TANK
        }
    }

    public static final class Climb {
        public static final int CLIMB_MOTOR_PORT = 0; // TODO: Change to correct port
    }

    public static final class EndEffector {
        public static final int INTAKE_MOTOR_PORT = 0; // TODO: Change to correct port
        public static final int SHOOTER_MOTOR_PORT = 0; // TODO: Change to correct port

        public static final double DEFAULT_INTAKE_SPEED = 1.0;
        public static final double DEFAULT_INTAKE_REVERSE_SPEED = -0.2;

        public static final double DEFAULT_SHOOTER_SPEED = 1.0;
        public static final double DEFAULT_SHOOTER_REVERSE_SPEED = -0.2;
    }
}