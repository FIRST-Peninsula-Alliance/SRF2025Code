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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAC;
import frc.robot.Constants.EC;
//import frc.robot.NotableConstants.IAC;

public class ElevatorSubsystem extends SubsystemBase {
  public final TalonFX m_elevatorMotor; 
  private double m_elevatorSetpoint;
  private final CANcoder m_elevatorCANCoder;
  private final double m_magnetOffset = EC.ELEVATOR_CANCODER_MAGNET_OFFSET;
  private final MotionMagicVoltage m_elevatorMagicCtrl = new MotionMagicVoltage(0.0)
                                                                                .withSlot(0)
                                                                                .withEnableFOC(true);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_elevatorMotor = new TalonFX(EC.ELEVATOR_MOTOR_CAN_ID);
    m_elevatorCANCoder = new CANcoder(EC.ELEVATOR_CANCODER_ID);

    configElevatorCANCoder();
    configElevatorMotor();
  }

  public void GoToPosition(double position) {
    m_elevatorSetpoint = position;
    m_elevatorMotor.setControl(m_elevatorMagicCtrl.withPosition(m_elevatorSetpoint));
  }

  public double getAbsCoralArmPos() {
    return(m_elevatorCANCoder.getAbsolutePosition().getValueAsDouble());
  }

  private void configElevatorCANCoder() {
    var magnetSensorConfigs = new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(CAC.CORAL_ARM_CANCODER_RANGE)
                                                       .withSensorDirection(CAC.CORAL_ARM_CANCODER_DIR)
                                                       .withMagnetOffset(m_magnetOffset);
    var ccConfig = new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs);
    StatusCode status = m_elevatorCANCoder.getConfigurator().apply(ccConfig);
    if (! status.isOK() ) {
      System.out.println("Failed to apply CORAL_CANcoder configs. Error code: "+status.toString());
    }
  }

  private void configElevatorMotor() {
    var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                        .withVoltageClosedLoopRampPeriod(EC.ELEVATOR_CLOSED_LOOP_RAMP_PERIOD)
                                                        .withTorqueClosedLoopRampPeriod(0);
    var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                                              .withFeedbackRemoteSensorID(EC.ELEVATOR_MOTOR_CAN_ID)
                                              .withSensorToMechanismRatio(EC.ELEVATOR_CANCODER_TO_AXLE_RATIO)
                                              .withRotorToSensorRatio(EC.ELEVATOR_ROTOR_TO_CANCODER_RATIO);
    var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(EC.ELEVATOR_MOTOR_NEUTRAL_MODE)
                                                    .withInverted(EC.ELEVATOR_MOTOR_INVERT)
                                                    .withPeakForwardDutyCycle(EC.ELEVATOR_OUTPUT_LIMIT_FACTOR)
                                                    .withPeakReverseDutyCycle(-EC.ELEVATOR_OUTPUT_LIMIT_FACTOR);
                                                    //.withDutyCycleNeutralDeadband(.001);
    var currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(EC.ELEVATOR_CONT_CURRENT_LIMIT)
                                                       .withSupplyCurrentLowerLimit(EC.ELEVATOR_PEAK_CURRENT_LIMIT)
                                                       .withSupplyCurrentLowerTime(EC.ELEVATOR_PEAK_CURRENT_DURATION)
                                                       .withSupplyCurrentLimitEnable(EC.ELEVATOR_ENABLE_CURRENT_LIMIT);
    Slot0Configs pid0Config = new Slot0Configs().withKP(EC.ELEVATOR_KP)
                                                .withKI(EC.ELEVATOR_KI)
                                                .withKD(EC.ELEVATOR_KD)
                                                .withKS(EC.ELEVATOR_KS)
                                                .withKV(EC.ELEVATOR_KV)
                                                .withKA(EC.ELEVATOR_KA)
                                                .withKG(EC.ELEVATOR_KG).withGravityType(GravityTypeValue.Arm_Cosine);
    MotionMagicConfigs  motionMagicConfig = new MotionMagicConfigs().withMotionMagicCruiseVelocity(EC.ELEVATOR_MOTION_MAGIC_VEL)
                                                                    .withMotionMagicAcceleration(EC.ELEVATOR_MOTION_MAGIC_ACCEL)
                                                                    .withMotionMagicJerk(EC.ELEVATOR_MOTION_MAGIC_JERK)
                                                                    .withMotionMagicExpo_kA(EC.ELEVATOR_MOTION_MAGIC_kA)
                                                                    .withMotionMagicExpo_kA(EC.ELEVATOR_MOTION_MAGIC_kV);
    var elevatorConfig = new TalonFXConfiguration().withFeedback(feedbackConfig)
                                                   .withMotorOutput(motorOutputConfig)
                                                   .withCurrentLimits(currentLimitConfig)
                                                   .withClosedLoopRamps(closedLoopConfig)
                                                   .withSlot0(pid0Config)
                                                   .withMotionMagic(motionMagicConfig);
    StatusCode status = m_elevatorMotor.getConfigurator().apply(elevatorConfig);

    if (! status.isOK() ) {
      System.out.println("Failed to apply ELEVATOR configs. Error code: "+status.toString());
    }
  }
  
  public void GoToCoralPickupPosition() {
    GoToPosition(EC.ELEVATOR_SOURCE_POSITION);
  }

  public void GoToL1CoralPosition() {
    GoToPosition(EC.ELEVATOR_L1_CORAL_POSITION);
  }

  public void GoToL2CoralPosition() {
    GoToPosition(EC.ELEVATOR_L2_CORAL_POSITION);
  }

  public void GoToL3CoralPosition() {
    GoToPosition(EC.ELEVATOR_L3_CORAL_POSITION);
  }

  public void GoToL2AlgaePosition() {
    GoToPosition(EC.ELEVATOR_L2_ALGAE_POSITION);
  }

  public void GoToL3AlgaePosition() {
    GoToPosition(EC.ELEVATOR_L3_ALGAE_POSITION);
  }

  public void GoToAlgaePickupPosition() {
    GoToPosition(EC.ELEVATOR_ALGAE_PICKUP_POSITION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
