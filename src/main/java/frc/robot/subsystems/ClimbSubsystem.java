// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

  private final TalonFX hookMotor;
  private final TalonFX armMotor;

  // private final MotionMagicVoltage armMotionMagic = new MotionMagicVoltage(0);
  private final MotionMagicVoltage hookMotionMagic = new MotionMagicVoltage(0);

  public ClimbSubsystem() {
    
    hookMotor = new TalonFX(Constants.ClimbConstants.HOOK_MOTOR_ID, Constants.ClimbConstants.HOOK_MOTOR_CANBUS);
    
    TalonFXConfiguration hookcfg = new TalonFXConfiguration();
    
    hookcfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hookcfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hookcfg.Feedback.SensorToMechanismRatio = Constants.ClimbConstants.HOOK_GEAR_RATIO;
    hookcfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    hookcfg.MotionMagic.MotionMagicCruiseVelocity = Constants.ClimbConstants.HOOK_MOTION_CRUISE_VELOCITY;
    hookcfg.MotionMagic.MotionMagicAcceleration = Constants.ClimbConstants.HOOK_MOTION_ACCELERATION;
    hookcfg.MotionMagic.MotionMagicJerk = Constants.ClimbConstants.HOOK_MOTION_JERK;

    hookcfg.CurrentLimits.SupplyCurrentLimit = 40.0;
    hookcfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    hookcfg.Slot0.kP = Constants.ClimbConstants.HOOK_kP;
    hookcfg.Slot0.kI = Constants.ClimbConstants.HOOK_kI;
    hookcfg.Slot0.kD = Constants.ClimbConstants.HOOK_kD;
    hookcfg.Slot0.kV = Constants.ClimbConstants.HOOK_kV;
    hookcfg.Slot0.kA = Constants.ClimbConstants.HOOK_kA;
    hookcfg.Slot0.kG = Constants.ClimbConstants.HOOK_kG;

    hookMotor.getConfigurator().apply(hookcfg);

    hookMotionMagic.Position = 0;

    armMotor = new TalonFX(Constants.ClimbConstants.ARM_MOTOR_ID, Constants.ClimbConstants.ARM_MOTOR_CANBUS);
    
    TalonFXConfiguration armcfg = new TalonFXConfiguration();

    armcfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armcfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armcfg.Feedback.SensorToMechanismRatio = Constants.ClimbConstants.ARM_GEAR_RATIO;
    armcfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // Arm is not velocity or position controlled, so we don't need motion magic or PID.

    // armcfg.MotionMagic.MotionMagicCruiseVelocity = Constants.ClimbConstants.ARM_MOTION_CRUISE_VELOCITY;
    // armcfg.MotionMagic.MotionMagicAcceleration = Constants.ClimbConstants.ARM_MOTION_ACCELERATION;
    // armcfg.MotionMagic.MotionMagicJerk = Constants.ClimbConstants.ARM_MOTION_JERK;

    // armcfg.Slot0.kP = Constants.ClimbConstants.ARM_kP;
    // armcfg.Slot0.kI = Constants.ClimbConstants.ARM_kI;
    // armcfg.Slot0.kD = Constants.ClimbConstants.ARM_kD;
    // armcfg.Slot0.kV = Constants.ClimbConstants.ARM_kV;
    // armcfg.Slot0.kA = Constants.ClimbConstants.ARM_kA;

    armMotor.getConfigurator().apply(armcfg);

    // armMotionMagic.Position = 0;
  }

  public void setHookPosition(double targetPosition) {
    double targetTicks = targetPosition * Constants.ClimbConstants.HOOK_GEAR_RATIO;
    hookMotionMagic.Position = targetTicks;
    hookMotor.setControl(hookMotionMagic);
  }

  public void stopHook() {
    hookMotor.stopMotor();
  }

  public void stopArm() {
    armMotor.stopMotor();
  }

  public void extendHook(){
    setHookPosition(Constants.ClimbConstants.HOOK_EXTENDED_POSITION);
  }

  public void retractHook(){
    setHookPosition(Constants.ClimbConstants.HOOK_RETRACTED_POSITION);
  }

  public void setArmPower(double armPower) {
    if (((armPower > 0) && (getArmPosition() < Constants.ClimbConstants.ARM_MAX)) || ((armPower < 0) && (getArmPosition() > Constants.ClimbConstants.ARM_MIN))){
      armMotor.setControl(new DutyCycleOut(armPower));
    }
  }

  public void raiseArm(){
    setArmPower(Constants.ClimbConstants.ARM_POWER);
  }

  public void lowerArm(){
    setArmPower(-Constants.ClimbConstants.ARM_POWER);
  }

  public void setHookPID(double kp, double ki, double kd){
      Slot0Configs slot0 = new Slot0Configs().withKP(kp).withKI(ki).withKD(kd);
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.Slot0 = slot0;
      hookMotor.getConfigurator().apply(config);
  }

  public double getHookPosition() {
    return hookMotor.getPosition().getValueAsDouble();
  }

  public double getArmPosition() {
    return armMotor.getPosition().getValueAsDouble();
  }

  public boolean hookExtended(){
    return (Math.abs(getHookPosition() - Constants.ClimbConstants.HOOK_EXTENDED_POSITION) < Constants.ClimbConstants.HOOK_ERROR_MARGIN);
  }

  public boolean hookRetracted() {
    return (Math.abs(getHookPosition() - Constants.ClimbConstants.HOOK_RETRACTED_POSITION) < Constants.ClimbConstants.HOOK_ERROR_MARGIN);
  }
}