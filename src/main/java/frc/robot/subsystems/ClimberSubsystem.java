// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.CSC;
import frc.robot.NotableConstants.CC;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public final TalonFX m_winchMotor;
  public final Servo m_hingeLinearServoMotor;
  public final Servo m_hookServoMotor;
  public final Servo m_springServoMotor;
  private DigitalInput m_climberLimitSwitch; 
  private DutyCycleOut m_climbRequest = new DutyCycleOut(CC.CLIMBER_DUTY_CYCLE)
                                                            .withEnableFOC(true)
                                                            .withUpdateFreqHz(50);
  private static double m_climbMotorFactor;
  private double m_climbStartTime;

  public ClimberSubsystem() {
    m_winchMotor = new TalonFX(CSC.WINCH_MOTOR_CAN_ID, Constants.CAN_BUS_IN_USE);
    m_hingeLinearServoMotor = new Servo(CSC.LINEAR_SERVO_PWM_CHANNEL);
    m_hookServoMotor = new Servo(CSC.HOOK_SERVO_PWM_CHANNEL);
    m_springServoMotor = new Servo(CSC.SPRING_SERVO_PWM_CHANNEL);
    m_climberLimitSwitch = new DigitalInput(1);

    configLinearServoMotor(m_hingeLinearServoMotor, "Hinge Pin Servo");
    configHookServoMotor(m_hookServoMotor, "Climber Hook Servo");
    configSpringServoMotor(m_springServoMotor, "Climber Gas Spring Servo");
    configWinchMotor();
  }

  public void configLinearServoMotor(Servo servo, String servoName) {
    servo.setPeriodMultiplier(PeriodMultiplier.k1X);
    servo.enableDeadbandElimination(true);
    //PWMConfigDataResult a = aGripServo.getRawBounds();
    //String out = " Max "+a.max+" DB Max "+a.deadbandMax+
    //             " Cen "+a.center+" DB Min "+a.deadbandMin+" Min "+a.min;
    //SmartDashboard.putString(servoName, out);
    //SmartDashboard.putNumber(servoName+"raw ", aGripServo.getRaw());
    //SmartDashboard.putNumber(servoName+"pos ", aGripServo.getPosition());
    SmartDashboard.putNumber(servoName+" spd ", servo.getSpeed());
    SmartDashboard.putNumber(servoName+" deg ", servo.getAngle());
  }

  public void configHookServoMotor(Servo servo, String servoName) {
    servo.setPeriodMultiplier(PeriodMultiplier.k1X);
    servo.enableDeadbandElimination(true);
    //PWMConfigDataResult a = aGripServo.getRawBounds();
    //String out = " Max "+a.max+" DB Max "+a.deadbandMax+
    //             " Cen "+a.center+" DB Min "+a.deadbandMin+" Min "+a.min;
    //SmartDashboard.putString(servoName, out);
    //SmartDashboard.putNumber(servoName+"raw ", aGripServo.getRaw());
    //SmartDashboard.putNumber(servoName+"pos ", aGripServo.getPosition());
    SmartDashboard.putNumber(servoName+" spd ", servo.getSpeed());
    SmartDashboard.putNumber(servoName+" deg ", servo.getAngle());
  }

  public void configSpringServoMotor(Servo servo, String servoName) {
    servo.setPeriodMultiplier(PeriodMultiplier.k1X);
    servo.enableDeadbandElimination(true);
    //PWMConfigDataResult a = aGripServo.getRawBounds();
    //String out = " Max "+a.max+" DB Max "+a.deadbandMax+
    //             " Cen "+a.center+" DB Min "+a.deadbandMin+" Min "+a.min;
    //SmartDashboard.putString(servoName, out);
    //SmartDashboard.putNumber(servoName+"raw ", aGripServo.getRaw());
    //SmartDashboard.putNumber(servoName+"pos ", aGripServo.getPosition());
    SmartDashboard.putNumber(servoName+" spd ", servo.getSpeed());
    SmartDashboard.putNumber(servoName+" deg ", servo.getAngle());
  }

  private void configWinchMotor() {
    var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                        .withVoltageClosedLoopRampPeriod(CSC.WINCH_CLOSED_LOOP_RAMP_PERIOD)
                                                        .withTorqueClosedLoopRampPeriod(0);
    var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                                              .withFeedbackRemoteSensorID(CSC.WINCH_MOTOR_CAN_ID)
                                              .withSensorToMechanismRatio(CSC.WINCH_CANCODER_TO_AXLE_RATIO)
                                              .withRotorToSensorRatio(CSC.WINCH_ROTOR_TO_CANCODER_RATIO);
    var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(CSC.WINCH_MOTOR_NEUTRAL_MODE)
                                                    .withInverted(CSC.WINCH_MOTOR_INVERT)
                                                    .withPeakForwardDutyCycle(CSC.WINCH_OUTPUT_LIMIT_FACTOR)
                                                    .withPeakReverseDutyCycle(-CSC.WINCH_OUTPUT_LIMIT_FACTOR);
                                                    //.withDutyCycleNeutralDeadband(.001);
    var currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(CSC.WINCH_CONT_CURRENT_LIMIT)
                                                       .withSupplyCurrentLowerLimit(CSC.WINCH_PEAK_CURRENT_LIMIT)
                                                       .withSupplyCurrentLowerTime(CSC.WINCH_PEAK_CURRENT_DURATION)
                                                       .withSupplyCurrentLimitEnable(CSC.WINCH_ENABLE_CURRENT_LIMIT);
    MotionMagicConfigs  motionMagicConfig = new MotionMagicConfigs().withMotionMagicCruiseVelocity(CSC.WINCH_MOTION_MAGIC_VEL)
                                                                    .withMotionMagicAcceleration(CSC.WINCH_MOTION_MAGIC_ACCEL)
                                                                    .withMotionMagicJerk(CSC.WINCH_MOTION_MAGIC_JERK)
                                                                    .withMotionMagicExpo_kA(CSC.WINCH_MOTION_MAGIC_kA)
                                                                    .withMotionMagicExpo_kA(CSC.WINCH_MOTION_MAGIC_kV);
    var coralArmConfig = new TalonFXConfiguration().withFeedback(feedbackConfig)
                                                   .withMotorOutput(motorOutputConfig)
                                                   .withCurrentLimits(currentLimitConfig)
                                                   .withClosedLoopRamps(closedLoopConfig)
                                                   .withMotionMagic(motionMagicConfig);
    StatusCode status = m_winchMotor.getConfigurator().apply(coralArmConfig);

    if (! status.isOK() ) {
      System.out.println("Failed to apply WINCH_MOTOR configs. Error code: "+status.toString());
    }
  }

  public void ReleaseClimberPin() {
    m_hingeLinearServoMotor.set(CSC.HINGE_SERVO_OPEN_POSITION);
  }

  public void EngageCLimberPin() {
    m_hingeLinearServoMotor.set(CSC.HINGE_SERVO_CLOSED_POSITION);
  }

  public void EngageHook() {
    m_hookServoMotor.set(CSC.HOOK_SERVO_CLOSED_POSITION);
  }

  public void DisengageHook() {
    m_hookServoMotor.set(CSC.HOOK_SERVO_OPEN_POSITION);
  }

  public void DisengageSpringServo() {
    m_springServoMotor.set(CSC.SPRING_SERVO_DISENGAGED_POSITION);
  }

  public void EngageSpringServo() {
    m_springServoMotor.set(CSC.SPRING_SERVO_ENGAGED_POSITION);
  }

  public void runWinch() {
    m_climbMotorFactor = 1.0;
    m_climbStartTime = System.currentTimeMillis();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if ((System.currentTimeMillis() - m_climbStartTime) > CSC.CLIMB_WINCH_TIMEOUT) {
      m_climbMotorFactor = 0.0;
    }

    m_winchMotor.setControl(m_climbRequest.withOutput(CC.CLIMBER_DUTY_CYCLE * 
                                                      m_climbMotorFactor)
                                                      .withEnableFOC(true)
                                                      .withUpdateFreqHz(50));

    if (m_climberLimitSwitch.get()) {
      EngageHook();
    }
  }
}
