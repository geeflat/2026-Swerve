// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

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

    public static final int kButtonBoardPort = 1;

    public static final int FIRE_ON_THE_MOVE_BUTTON = 1;
    public static final int PUSH_BUTTON = 2;
    public static final int CLIMB_BUTTON = 3;
    public static final int IDLE_BUTTON = 4;
    public static final int UNJAM_BUTTON = 5;
  }

  public static class IntakeConstants {
    public static final int intakeMotor1ID = 0;
    public static final String intakemotor1CANBus = "canivore1";
    public static final double INTAKE_SPEED = 0.3;
  }

  public static class ShooterConstants {
    public static final int shooterMotor1ID = 1;
    public static final String shootermotor1CANBus = "canivore";

    public static final int shootermotor2ID = 7;
    public static final String shootermotor2CANBus = "canivore";

    public static final double SHOOTER_SPEED = 0.5;

    public static final double kS = 0.2; // Static gain (volts)
    public static final double kP = 0.1; // Proportional gain (volts per unit error)
    public static final double kV = 0.1; // Velocity gain (volts per unit velocity)
    public static final double kI = 0.0; // Integral gain (volts per unit integral)
    public static final double kD = 0.0; // Derivative gain (volts per unit derivative)

    public static final double FEED_FORWARD = 0.05; // Additional feedforward voltage to help overcome static friction

    public static final boolean INVERT_FOLLOWER = true;
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

    public static final int ENCODER_TICKS_PER_REVOLUTION = 2048;
    public static final double SLEW_RATE_LIMITER = 3.0; // units per second
  }

  public static class IndexConstants {
    public static final int horizontalIndexMotorID = 4;
    public static final String horizontalIndexMotorCANBus = "canivore";

    public static final int verticalIndexMotorID = 5;
    public static final String verticalIndexMotorCANBus = "canivore";

    public static final int verticalIndexFollowerID = 6;
    public static final String verticalIndexFollowerCANBus = "canivore";

    public static final boolean INVERT_FOLLOWER = false;

    public static final double HORIZONTAL_INDEX_SPEED = 0.3;
    public static final double VERTICAL_INDEX_SPEED = 0.3;

    public static final double INDEX_CURRENT_LIMIT = 30.0; // amps
    public static final boolean INDEX_CURRENT_LIMIT_ENABLE = true;
   }

   public static class TurretConstants {
      public static final int turretMotorID = 6;
      public static final String turretMotorCANBus = "canivore";

      public static final double kP = 0.1;
      public static final double kI = 0.0; 
      public static final double kD = 0.0;
      public static final double kV = 0.0;
      public static final double kG = 0.0;

      public static final double MOTION_CRUISE_VELOCITY = 600;
      public static final double MOTION_ACCELERATION = 1200;

      public static final double ENCODER_TICKS_PER_REVOLUTION = 2048;
      public static final double SLEW_RATE_LIMITER = 3.0; // units per second

      public static final double MOTOR_TOOTH_COUNT = 22.0;
      public static final double TURRET_TOOTH_COUNT = 120.0;
   }

   public static class HoodConstants {
      public static final int hoodMotorID = 7;
      public static final String hoodMotorCANBus = "canivore";

      public static final double kP = 0.1;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kV = 0.0;
      public static final double kG = 0.0;

      public static final double MOTION_CRUISE_VELOCITY = 600;
      public static final double MOTION_ACCELERATION = 1200;

      public static final double UPPER_LIMIT = 90;
      public static final double LOWER_LIMIT = 40;

      public static final double POSITION_TOLERANCE_DEGREES = 1.0;
      public static final double MAX_CURRENT_AMPS = 40.0;

      public static final int ENCODER_TICKS_PER_REVOLUTION = 2048;
      public static final double SLEW_RATE_LIMITER = 3.0; // units per second
   }

   public static class robotStates {
      public enum State{
        FIRE_ON_THE_MOVE, // intake, index, and shoot on the move, disable climb motors
        PUSH,             // motor problems, switch to defensive mode
        CLIMB,            // active during endgame - disengage other motors and engage climb motors
        IDLE              // default state, all motors off
      }
  }
}
