// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AAC;
//import frc.robot.NotableConstants.IAC;

public class AlgaeArmSubsystem extends SubsystemBase {
  /** Creates a new algaeArmSubsystem. */
  public final TalonFX m_algaeArmMotor; 
  public final TalonSRX m_algaeWheelMotor;
  private double m_algaeArmSetpoint;
  private final MotionMagicVoltage m_algaeArmMagicCtrl = new MotionMagicVoltage(0.0)
                                                                                .withSlot(0)
                                                                                .withEnableFOC(true);

  public AlgaeArmSubsystem() {
    m_algaeArmMotor = new TalonFX(AAC.ALGAE_MOTOR_CAN_ID, Constants.CAN_BUS_IN_USE);
    m_algaeWheelMotor = new TalonSRX(AAC.ALGAE_WHEEL_MOTOR_CAN_ID);
    configAlgaeWheelMotor();
    configAlgaeArmMotor();
  }

  public void GoToPosition(double position) {
    m_algaeArmSetpoint = position;
    m_algaeArmMotor.setControl(m_algaeArmMagicCtrl.withPosition(m_algaeArmSetpoint));
  }

  private void configAlgaeWheelMotor() {
    m_algaeWheelMotor.configFactoryDefault();
    m_algaeWheelMotor.configSupplyCurrentLimit(AAC.ALGAE_WHEEL_CURRENT_LIMIT);
    m_algaeWheelMotor.setInverted(true);
    m_algaeWheelMotor.setNeutralMode(NeutralMode.Brake);
    m_algaeWheelMotor.configPeakOutputForward(AAC.ALGAE_WHEEL_OUTPUT_LIMIT_FACTOR);
    m_algaeWheelMotor.configPeakOutputReverse(-AAC.ALGAE_WHEEL_OUTPUT_LIMIT_FACTOR);
  }

  private void configAlgaeArmMotor() {
    var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                        .withVoltageClosedLoopRampPeriod(AAC.ALGAE_ARM_CLOSED_LOOP_RAMP_PERIOD)
                                                        .withTorqueClosedLoopRampPeriod(0);
    var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                                              .withFeedbackRemoteSensorID(AAC.ALGAE_MOTOR_CAN_ID)
                                              .withSensorToMechanismRatio(AAC.ALGAE_ARM_CANCODER_TO_AXLE_RATIO)
                                              .withRotorToSensorRatio(AAC.ALGAE_ARM_ROTOR_TO_CANCODER_RATIO);
    var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(AAC.ALGAE_ARM_MOTOR_NEUTRAL_MODE)
                                                    .withInverted(AAC.ALGAE_ARM_MOTOR_INVERT)
                                                    .withPeakForwardDutyCycle(AAC.ALGAE_ARM_OUTPUT_LIMIT_FACTOR)
                                                    .withPeakReverseDutyCycle(-AAC.ALGAE_ARM_OUTPUT_LIMIT_FACTOR);
                                                    //.withDutyCycleNeutralDeadband(.001);
    var currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(AAC.ALGAE_ARM_CONT_CURRENT_LIMIT)
                                                       .withSupplyCurrentLowerLimit(AAC.ALGAE_ARM_PEAK_CURRENT_LIMIT)
                                                       .withSupplyCurrentLowerTime(AAC.ALGAE_ARM_PEAK_CURRENT_DURATION)
                                                       .withSupplyCurrentLimitEnable(AAC.ALGAE_ARM_ENABLE_CURRENT_LIMIT);
    Slot0Configs pid0Config = new Slot0Configs().withKP(AAC.ALGAE_ARM_KP)
                                                .withKI(AAC.ALGAE_ARM_KI)
                                                .withKD(AAC.ALGAE_ARM_KD)
                                                .withKS(AAC.ALGAE_ARM_KS)
                                                .withKV(AAC.ALGAE_ARM_KV)
                                                .withKA(AAC.ALGAE_ARM_KA)
                                                .withKG(AAC.ALGAE_ARM_KG).withGravityType(GravityTypeValue.Arm_Cosine);
    MotionMagicConfigs  motionMagicConfig = new MotionMagicConfigs().withMotionMagicCruiseVelocity(AAC.ALGAE_ARM_MOTION_MAGIC_VEL)
                                                                    .withMotionMagicAcceleration(AAC.ALGAE_ARM_MOTION_MAGIC_ACCEL)
                                                                    .withMotionMagicJerk(AAC.ALGAE_ARM_MOTION_MAGIC_JERK)
                                                                    .withMotionMagicExpo_kA(AAC.ALGAE_ARM_MOTION_MAGIC_kA)
                                                                    .withMotionMagicExpo_kA(AAC.ALGAE_ARM_MOTION_MAGIC_kV);
    var algaeArmConfig = new TalonFXConfiguration().withFeedback(feedbackConfig)
                                                   .withMotorOutput(motorOutputConfig)
                                                   .withCurrentLimits(currentLimitConfig)
                                                   .withClosedLoopRamps(closedLoopConfig)
                                                   .withSlot0(pid0Config)
                                                   .withMotionMagic(motionMagicConfig);
    StatusCode status = m_algaeArmMotor.getConfigurator().apply(algaeArmConfig);

    if (! status.isOK() ) {
      System.out.println("Failed to apply ALGAE_ARM configs. Error code: "+status.toString());
    }
  }

  public void GoToL2RemovePosition() {
    GoToPosition(AAC.ALGAE_ARM_L2_POSITION);
  }

  public void GoToL3RemovePosition() {
    GoToPosition(AAC.ALGAE_ARM_L3_POSITION);
  }

  public void GoToPickupPosition() {
    GoToPosition(AAC.ALGAE_ARM_PICKUP_POSITION);
  }

  public void RunAlgaeWheels() {
    m_algaeWheelMotor.set(ControlMode.PercentOutput, .5);
  }

  public void StopWheels() {
    m_algaeWheelMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
