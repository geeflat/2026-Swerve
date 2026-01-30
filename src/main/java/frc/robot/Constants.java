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
    public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;
  }
  public static class IntakeConstants {
    public static final int intakeMotor1ID = 0;
    public static final String intakemotor1CANBus = "canivore1";
    public static final double intakeSpeed = 0.3;
  }
  public static class ShooterConstants {
    public static final int shooterMotor1ID = 1;
    public static final String shootermotor1CANBus = "canivore";
    public static final double shooterSpeed = 0.5;
  }
  public static class IntakeExtenderConstants{
    public static final int leaderExtenderID = 2;
    public static final String leaderExtenderCANBus = "canivore";
    public static final int followerExtenderID = 3;
    public static final String followerExtenderCANBus = "canivore";

    public static final int IntakeGearRatio = 12;

    public static final int LEFT_ENCODER_DIO = 0;   // DIO port for left encoder (left = leader)
    public static final int RIGHT_ENCODER_DIO = 1;  // DIO port for right encoder

    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.0;
    public static final double kG = 0.0;

    public static final double MOTION_CRUISE_VELOCITY = 600;
    public static final double MOTION_ACCELERATION = 1200;

    public static final double UP_POSITION_DEGREES = 0;
    public static final double DOWN_POSITION_DEGREES = 90;

    public static final double POSITION_TOLERANCE_DEGREES = 2.0;
    public static final double MAX_CURRENT_AMPS = 40.0;

    public static final boolean INVERT_LEADER_MOTOR = false;
    public static final boolean INVERT_FOLLOWER_MOTOR = true;

    public static final int EncoderTicksPerRevolution = 2048;
    public static final double SlewRateLimiter = 3.0; // units per second
  }

}
