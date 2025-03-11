// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.CSC;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public final TalonFX m_winchMotor;
  public final Servo m_hingeLinearServoMotor;
  public final Servo m_hookServoMotor;
  public final Servo m_springServoMotor;
  private DigitalInput m_cradleLimitSwitch; 
  private DigitalInput m_winchLimitSwitch; 
  private PositionVoltage m_climbRequest = new PositionVoltage(0)
                                                               .withEnableFOC(true)
                                                               .withSlot(0);
  private DutyCycleOut m_testClimbRequest = new DutyCycleOut(-1)
                                                            .withEnableFOC(true);
  private double m_springReleaseServoIncrement;
  private double m_linearServoIncrement;
  private double m_hookServoIncrement;

  public ClimberSubsystem() {
    m_winchMotor = new TalonFX(CSC.WINCH_MOTOR_CAN_ID, Constants.CAN_BUS_IN_USE);
    m_hingeLinearServoMotor = new Servo(CSC.LINEAR_SERVO_PWM_CHANNEL);
    m_hookServoMotor = new Servo(CSC.HOOK_SERVO_PWM_CHANNEL);
    m_springServoMotor = new Servo(CSC.SPRING_SERVO_PWM_CHANNEL);
    m_cradleLimitSwitch = new DigitalInput(0);
    m_winchLimitSwitch = new DigitalInput(1);

    configServoMotor(m_hingeLinearServoMotor, "Hinge Pin Servo", CSC.HINGE_SERVO_CLOSED_POSITION);
    configServoMotor(m_hookServoMotor, "Climber Hook Servo", CSC.HOOK_SERVO_OPEN_POSITION);
    configServoMotor(m_springServoMotor, "Climber Gas Spring Servo", CSC.SPRING_SERVO_ENGAGED_POSITION);
    configWinchMotor();
  }

  public void configServoMotor(Servo servo, String servoName, double servoInitPos) {
    servo.setPeriodMultiplier(PeriodMultiplier.k1X);
    servo.enableDeadbandElimination(true);
    servo.set(servoInitPos);
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
    var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                              .withFeedbackRemoteSensorID(CSC.WINCH_MOTOR_CAN_ID)
                                              .withSensorToMechanismRatio(CSC.WINCH_MOTOR_TO_AXLE_RATIO)
                                              .withRotorToSensorRatio(CSC.WINCH_ROTOR_TO_CANCODER_RATIO);
    var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(CSC.WINCH_MOTOR_NEUTRAL_MODE)
                                                    .withInverted(CSC.WINCH_MOTOR_INVERT)
                                                    .withPeakForwardDutyCycle(CSC.WINCH_OUTPUT_LIMIT_FACTOR)
                                                    .withPeakReverseDutyCycle(-CSC.WINCH_OUTPUT_LIMIT_FACTOR);
                                                    //.withDutyCycleNeutralDeadband(.001);
    var currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(CSC.WINCH_CONT_CURRENT_LIMIT)
                                                       .withSupplyCurrentLowerLimit(CSC.WINCH_PEAK_CURRENT_LIMIT)
                                                       .withSupplyCurrentLowerTime(CSC.WINCH_PEAK_CURRENT_DURATION)
                                                       .withSupplyCurrentLimitEnable(CSC.WINCH_ENABLE_CURRENT_LIMIT)
                                                       .withStatorCurrentLimit(CSC.WINCH_STATOR_CURRENT_LIMIT)
                                                       .withStatorCurrentLimitEnable(CSC.WINCH_ENABLE_CURRENT_LIMIT);
    Slot0Configs pid0Config = new Slot0Configs().withKP(CSC.WINCH_KP)
                                                .withKI(CSC.WINCH_KI)
                                                .withKD(CSC.WINCH_KD)
                                                .withKS(CSC.WINCH_KS)
                                                .withKV(CSC.WINCH_KV)
                                                .withKA(CSC.WINCH_KA);
    var winchConfig = new TalonFXConfiguration().withFeedback(feedbackConfig)
                                                   .withSlot0(pid0Config)
                                                   .withMotorOutput(motorOutputConfig)
                                                   .withCurrentLimits(currentLimitConfig)
                                                   .withClosedLoopRamps(closedLoopConfig);
    StatusCode status = m_winchMotor.getConfigurator().apply(winchConfig);

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

  public void IncrementSpringServo() {
    m_springReleaseServoIncrement = m_springReleaseServoIncrement + 0.1;
    m_springServoMotor.set(m_springReleaseServoIncrement);
  }

  public void DecrementSpringServo() {
    m_springReleaseServoIncrement = m_springReleaseServoIncrement - 0.1;
    m_springServoMotor.set(m_springReleaseServoIncrement);
  }

  public void IncrementHookServo() {
    m_hookServoIncrement = m_hookServoIncrement + 0.1;
    m_hookServoMotor.set(m_hookServoIncrement);
  }

  public void DecrementHookServo() {
    m_hookServoIncrement = m_hookServoIncrement - 0.1;
    m_hookServoMotor.set(m_hookServoIncrement);
  }

  public void IncrementLinearServo() {
    m_linearServoIncrement = m_linearServoIncrement + 0.1;
    m_hingeLinearServoMotor.set(m_linearServoIncrement);
  }

  public void DecrementLinearServo() {
    m_linearServoIncrement = m_linearServoIncrement - 0.1;
    m_hingeLinearServoMotor.set(m_linearServoIncrement);
  }

  public void runWinch() {
    m_winchMotor.setControl(m_climbRequest.withPosition(CSC.WINCH_SETPOINT));  
  }

  public void TestWinch() {
    m_winchMotor.setControl(m_testClimbRequest);
  }

  public void StopWinch() {
    m_winchMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Timer.getMatchTime() < 40) {
      if (m_cradleLimitSwitch.get() == false) {
      EngageHook();
      }

      if (m_winchLimitSwitch.get() == false) {
      StopWinch();
      }
    }

    //if (System.currentTimeMillis() - m_climbStartTime > 2500) {
    //  StopWinch();
    //}


    SmartDashboard.putBoolean("Hook Engaged", !m_cradleLimitSwitch.get());
    SmartDashboard.putBoolean("Winch Stopped", !m_winchLimitSwitch.get());
    SmartDashboard.putNumber("HookServoPosition", m_hookServoIncrement);
    SmartDashboard.putNumber("LinearServoPosition", m_linearServoIncrement);
    SmartDashboard.putNumber("SpringServoPosition", m_springReleaseServoIncrement);
    SmartDashboard.putNumber("WinchMotorCounts", m_winchMotor.getPosition().getValueAsDouble());
  }
}
