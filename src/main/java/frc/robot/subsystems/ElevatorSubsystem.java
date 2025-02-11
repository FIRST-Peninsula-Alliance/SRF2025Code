// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EC;
import frc.robot.NotableConstants.IAC;

public class ElevatorSubsystem extends SubsystemBase {
  public final TalonFX m_elevatorArmMotor;
  public final CANcoder m_elevatorArmCANcoder; 
  private double m_elevatorArmSetpoint;
  private final MotionMagicVoltage m_elevatorArmMagicCtrl = new MotionMagicVoltage(0.0)
                                                                                .withSlot(0)
                                                                                .withEnableFOC(true);
  private double m_magnetOffset = IAC.INNER_ARM_CANCODER_MAGNET_OFFSET;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_elevatorArmMotor = new TalonFX(EC.ELEVATOR_MOTOR_CAN_ID);
    m_elevatorArmCANcoder = new CANcoder(EC.ELEVATOR_CANCODER_ID, Constants.CANIVORE_BUS_NAME);

    configElevatorArmCANcoder();
    configElevatorArmMotor();
  }

  public void GoToPosition(double position) {
    m_elevatorArmSetpoint = position;
    m_elevatorArmMotor.setControl(m_elevatorArmMagicCtrl.withPosition(m_elevatorArmSetpoint));
  }

  public double getAbsCoralArmPos() {
    return(m_elevatorArmCANcoder.getAbsolutePosition().getValueAsDouble());
  }

  private void configElevatorArmCANcoder() {
    var magnetSensorConfigs = new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(EC.ELEVATOR_ARM_CANCODER_RANGE)
                                                       .withSensorDirection(EC.ELEVATOR_ARM_CANCODER_DIR)
                                                       .withMagnetOffset(m_magnetOffset);
    var ccConfig = new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs);
    StatusCode status = m_elevatorArmCANcoder.getConfigurator().apply(ccConfig);
    if (! status.isOK() ) {
      System.out.println("Failed to apply INNER_CANcoder configs. Error code: "+status.toString());
    }
  }

  private void configElevatorArmMotor() {
    var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                        .withVoltageClosedLoopRampPeriod(EC.ELEVATOR_ARM_CLOSED_LOOP_RAMP_PERIOD)
                                                        .withTorqueClosedLoopRampPeriod(0);
    var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                                              .withFeedbackRemoteSensorID(EC.ELEVATOR_MOTOR_CAN_ID)
                                              .withSensorToMechanismRatio(EC.ELEVATOR_ARM_CANCODER_TO_AXLE_RATIO)
                                              .withRotorToSensorRatio(EC.ELEVATOR_ARM_ROTOR_TO_CANCODER_RATIO);
    var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(EC.ELEVATOR_ARM_MOTOR_NEUTRAL_MODE)
                                                    .withInverted(EC.ELEVATOR_ARM_MOTOR_INVERT)
                                                    .withPeakForwardDutyCycle(EC.ELEVATOR_ARM_OUTPUT_LIMIT_FACTOR)
                                                    .withPeakReverseDutyCycle(-EC.ELEVATOR_ARM_OUTPUT_LIMIT_FACTOR);
                                                    //.withDutyCycleNeutralDeadband(.001);
    var currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(EC.ELEVATOR_ARM_CONT_CURRENT_LIMIT)
                                                       .withSupplyCurrentLowerLimit(EC.ELEVATOR_ARM_PEAK_CURRENT_LIMIT)
                                                       .withSupplyCurrentLowerTime(EC.ELEVATOR_ARM_PEAK_CURRENT_DURATION)
                                                       .withSupplyCurrentLimitEnable(EC.ELEVATOR_ARM_ENABLE_CURRENT_LIMIT);
    Slot0Configs pid0Config = new Slot0Configs().withKP(EC.ELEVATOR_ARM_KP)
                                                .withKI(EC.ELEVATOR_ARM_KI)
                                                .withKD(EC.ELEVATOR_ARM_KD)
                                                .withKS(EC.ELEVATOR_ARM_KS)
                                                .withKV(EC.ELEVATOR_ARM_KV)
                                                .withKA(EC.ELEVATOR_ARM_KA)
                                                .withKG(EC.ELEVATOR_ARM_KG).withGravityType(GravityTypeValue.Arm_Cosine);
    MotionMagicConfigs  motionMagicConfig = new MotionMagicConfigs().withMotionMagicCruiseVelocity(EC.ELEVATOR_ARM_MOTION_MAGIC_VEL)
                                                                    .withMotionMagicAcceleration(EC.ELEVATOR_ARM_MOTION_MAGIC_ACCEL)
                                                                    .withMotionMagicJerk(EC.ELEVATOR_ARM_MOTION_MAGIC_JERK)
                                                                    .withMotionMagicExpo_kA(EC.ELEVATOR_ARM_MOTION_MAGIC_kA)
                                                                    .withMotionMagicExpo_kA(EC.ELEVATOR_ARM_MOTION_MAGIC_kV);
    var elevatorArmConfig = new TalonFXConfiguration().withFeedback(feedbackConfig)
                                                   .withMotorOutput(motorOutputConfig)
                                                   .withCurrentLimits(currentLimitConfig)
                                                   .withClosedLoopRamps(closedLoopConfig)
                                                   .withSlot0(pid0Config)
                                                   .withMotionMagic(motionMagicConfig);
    StatusCode status = m_elevatorArmMotor.getConfigurator().apply(elevatorArmConfig);

    if (! status.isOK() ) {
      System.out.println("Failed to apply INNER_ARM configs. Error code: "+status.toString());
    }
  }


  public void GoToSourcePosition() {
    GoToPosition(EC.ELEVATOR_ARM_SOURCE_POSITION);
  }

  public void GoToScorePosition() {
    GoToPosition(EC.ELEVATOR_ARM_SCORE_POSITION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
