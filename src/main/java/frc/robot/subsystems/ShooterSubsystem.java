// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.ShuffleboardControl;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
private TalonFX shooterMotor1;

public ShooterSubsystem() {

    shooterMotor1 = new TalonFX(Constants.ShooterConstants.shooterMotor1ID, Constants.ShooterConstants.shootermotor1CANBus);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = Constants.ShooterConstants.kS;
    slot0Configs.kV = Constants.ShooterConstants.kV;
    slot0Configs.kP = Constants.ShooterConstants.kP;
    slot0Configs.kI = Constants.ShooterConstants.kI;
    slot0Configs.kD = Constants.ShooterConstants.kD;
    
    shooterMotor1.getConfigurator().apply(slot0Configs);
    
    // register ShooterMotor1 as a continuous motor in Shuffleboard

    ShuffleboardControl.registerContinuousMotor(
        "Shooter Motor",
        Constants.ShooterConstants.shooterMotor1ID,
        Constants.ShooterConstants.shootermotor1CANBus,
        new ShuffleboardControl.MotorAccessor() {
          private final DutyCycleOut request = new DutyCycleOut(0.0);

          @Override
          public void setPower(double power) {
              // -1.0 to 1.0 → DutyCycleOut uses -1 to 1 directly
              request.Output = power;
              shooterMotor1.setControl(request);
          }

          @Override
          public double getPower() {
              // Best: get the applied duty cycle (what the motor is actually outputting)
              return shooterMotor1.getDutyCycle().getValueAsDouble();
          }
          @Override public void setPosition(double pos) { /* not used */ }
          @Override public double getPosition() { return 0; }
        }
      );
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void shoot(){

    final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

    shooterMotor1.setControl(request.withVelocity(Constants.ShooterConstants.SHOOTER_SPEED).withFeedForward(Constants.ShooterConstants.FEED_FORWARD));
  }

  public void stop(){
    shooterMotor1.setControl(new DutyCycleOut(0.0));
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    shoot();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
