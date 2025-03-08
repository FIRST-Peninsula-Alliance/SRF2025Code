// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ExternalFeedbackConfigs;
//import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
//import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
//import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAC;
//import frc.robot.NotableConstants.IAC;
import frc.robot.NotableConstants.IAC;

public class CoralArmSubsystem extends SubsystemBase {
  /** Creates a new CoralArmSubsystem. 
  public final TalonFXS m_coralArmMotor;
  public final CANcoder m_coralArmCANcoder; 
  public final Servo m_pinServoMotor;
  private double m_coralArmSetpoint;
  private final PositionVoltage m_positionRequest = new PositionVoltage(0.0)
                                                                        .withSlot(0)
                                                                        .withEnableFOC(true);
  private final MotionMagicVoltage m_coralArmMagicCtrl = new MotionMagicVoltage(0.0)
                                                                                .withSlot(0)
                                                                                .withEnableFOC(true);
  private double m_magnetOffset = CAC.CORAL_ARM_CANCODER_MAGNET_OFFSET;

  public CoralArmSubsystem() {
    m_coralArmMotor = new TalonFXS(CAC.CORAL_MOTOR_CAN_ID, Constants.CAN_BUS_IN_USE);
    m_coralArmCANcoder = new CANcoder(CAC.CORAL_CANCODER_ID, Constants.CAN_BUS_IN_USE);
    m_pinServoMotor = new Servo(CAC.CORAL_ARM_SERVO_PWM_CHANNEL);

    configPinServoMotor(m_pinServoMotor, "Collector Pin Servo");
    configCoralArmCANcoder();
    configCoralArmMotor();

    GoToCenterPosition();
  }

  public void GoToPosition(double position) {
    m_coralArmSetpoint = position;
    m_coralArmMotor.setControl(m_positionRequest.withPosition(m_coralArmSetpoint));
  }

  public double getAbsCoralArmPos() {
    return(m_coralArmCANcoder.getAbsolutePosition().getValueAsDouble());
  }

  private void configCoralArmCANcoder() {
    var magnetSensorConfigs = new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(CAC.CORAL_ARM_CANCODER_RANGE)
                                                       .withSensorDirection(CAC.CORAL_ARM_CANCODER_DIR)
                                                       .withMagnetOffset(m_magnetOffset);
    var ccConfig = new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs);
    StatusCode status = m_coralArmCANcoder.getConfigurator().apply(ccConfig);
    if (! status.isOK() ) {
      System.out.println("Failed to apply CORAL_CANcoder configs. Error code: "+status.toString());
    }
  }

 private void configCoralArmMotor() {
    var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                        .withVoltageClosedLoopRampPeriod(CAC.CORAL_ARM_CLOSED_LOOP_RAMP_PERIOD)
                                                        .withTorqueClosedLoopRampPeriod(0);
    var commutationConfig = new CommutationConfigs().withAdvancedHallSupport(AdvancedHallSupportValue.Enabled)
                                                    .withMotorArrangement(MotorArrangementValue.NEO_JST);
    var externalFeedbackConfig = new ExternalFeedbackConfigs().withFusedCANcoder(m_coralArmCANcoder)
                                                              .withSensorToMechanismRatio(CAC.CORAL_ARM_MECHANISM_RATIO);
    var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(CAC.CORAL_ARM_MOTOR_NEUTRAL_MODE)
                                                    .withInverted(CAC.CORAL_ARM_MOTOR_INVERT)
                                                    .withPeakForwardDutyCycle(CAC.CORAL_ARM_OUTPUT_LIMIT_FACTOR)
                                                    .withPeakReverseDutyCycle(-CAC.CORAL_ARM_OUTPUT_LIMIT_FACTOR);
                                                    //.withDutyCycleNeutralDeadband(.001);
    var currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(CAC.CORAL_ARM_CONT_CURRENT_LIMIT)
                                                       .withSupplyCurrentLowerLimit(CAC.CORAL_ARM_PEAK_CURRENT_LIMIT)
                                                       .withSupplyCurrentLowerTime(CAC.CORAL_ARM_PEAK_CURRENT_DURATION)
                                                       .withSupplyCurrentLimitEnable(CAC.CORAL_ARM_ENABLE_CURRENT_LIMIT)
                                                       .withStatorCurrentLimit(14);
    var softwareLimitSwitchConfig = new SoftwareLimitSwitchConfigs().withForwardSoftLimitThreshold(0) //TODO: add to constants
                                                                    .withForwardSoftLimitEnable(true)
                                                                    .withReverseSoftLimitThreshold(-0.4)
                                                                    .withReverseSoftLimitEnable(true);
    Slot0Configs pid0Config = new Slot0Configs().withKP(CAC.CORAL_ARM_KP)
                                                .withKI(CAC.CORAL_ARM_KI)
                                                .withKD(CAC.CORAL_ARM_KD)
                                                .withKS(CAC.CORAL_ARM_KS)
                                                .withKV(CAC.CORAL_ARM_KV)
                                                .withKA(CAC.CORAL_ARM_KA)
                                                .withKG(CAC.CORAL_ARM_KG).withGravityType(GravityTypeValue.Arm_Cosine);
    MotionMagicConfigs  motionMagicConfig = new MotionMagicConfigs().withMotionMagicCruiseVelocity(CAC.CORAL_ARM_MOTION_MAGIC_VEL)
                                                                    .withMotionMagicAcceleration(CAC.CORAL_ARM_MOTION_MAGIC_ACCEL)
                                                                    .withMotionMagicJerk(CAC.CORAL_ARM_MOTION_MAGIC_JERK)
                                                                    .withMotionMagicExpo_kA(CAC.CORAL_ARM_MOTION_MAGIC_kA)
                                                                    .withMotionMagicExpo_kA(CAC.CORAL_ARM_MOTION_MAGIC_kV);
    var coralArmConfig = new TalonFXSConfiguration().withMotorOutput(motorOutputConfig)
                                                   .withCurrentLimits(currentLimitConfig)
                                                   .withClosedLoopRamps(closedLoopConfig)
                                                   .withSlot0(pid0Config)
                                                   .withMotionMagic(motionMagicConfig)
                                                   .withCommutation(commutationConfig)
                                                   .withExternalFeedback(externalFeedbackConfig);
    StatusCode status = m_coralArmMotor.getConfigurator().apply(coralArmConfig);

    if (! status.isOK() ) {
      System.out.println("Failed to apply CORAL_ARM configs. Error code: "+status.toString());
    }
    SmartDashboard.putNumber("Coral Arm Motor Angle ROT", m_coralArmCANcoder.getAbsolutePosition().getValueAsDouble());
  }

  public void configPinServoMotor(Servo servo, String servoName) {
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

  public void GoToSourcePosition() {
    ResetPin();
    GoToPosition(CAC.CORAL_ARM_SOURCE_POSITION);
  }

  public void GoToScorePosition() {
    GoToPosition(CAC.CORAL_ARM_SCORE_POSITION);
  }
  public void GoToCenterPosition() { 
    ResetPin();
    GoToPosition(CAC.CORAL_ARM_CENTER_POSITION);
  }
  
  public void ScoreCoral() {
    m_pinServoMotor.setPosition(CAC.PIN_SERVO_OPEN_POSITION);
  } 

  public void ResetPin() {
    m_pinServoMotor.setPosition(CAC.PIN_SERVO_CLOSED_POSITION);
  }

  public void ScoreL1Position() {
    GoToPosition(CAC.CORAL_ARM_SCORE_L1_POSITION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Coral Arm CANcoder position", m_coralArmCANcoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Coral Arm Servo Position", m_pinServoMotor.getPosition());
  }
} */
