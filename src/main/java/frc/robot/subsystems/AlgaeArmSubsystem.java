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
import frc.robot.Constants.AAC;
import frc.robot.NotableConstants.IAC;

public class AlgaeArmSubsystem extends SubsystemBase {
  /** Creates a new algaeArmSubsystem. */
  public final TalonFX m_algaeArmMotor;
  public final CANcoder m_algaeArmCANcoder; 
  public final Servo m_pinServoMotor;
  private double m_algaeArmSetpoint;
  private final MotionMagicVoltage m_algaeArmMagicCtrl = new MotionMagicVoltage(0.0)
                                                                                .withSlot(0)
                                                                                .withEnableFOC(true);
  private double m_magnetOffset = IAC.INNER_ARM_CANCODER_MAGNET_OFFSET;

  public AlgaeArmSubsystem() {
    m_algaeArmMotor = new TalonFX(AAC.ALGAE_MOTOR_CAN_ID);
    m_algaeArmCANcoder = new CANcoder(AAC.ALGAE_CANCODER_ID, Constants.CANIVORE_BUS_NAME);
    m_pinServoMotor = new Servo(AAC.ALGAE_ARM_SERVO_CAN_ID);

    configPinServoMotor(m_pinServoMotor, "Collector Pin Servo");
    configalgaeArmCANcoder();
    configalgaeArmMotor();
  }

  public void GoToPosition(double position) {
    m_algaeArmSetpoint = position;
    m_algaeArmMotor.setControl(m_algaeArmMagicCtrl.withPosition(m_algaeArmSetpoint));
  }

  public double getAbsAlgaeArmPos() {
    return(m_algaeArmCANcoder.getAbsolutePosition().getValueAsDouble());
  }

  private void configalgaeArmCANcoder() {
    var magnetSensorConfigs = new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(AAC.ALGAE_ARM_CANCODER_RANGE)
                                                       .withSensorDirection(AAC.ALGAE_ARM_CANCODER_DIR)
                                                       .withMagnetOffset(m_magnetOffset);
    var ccConfig = new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs);
    StatusCode status = m_algaeArmCANcoder.getConfigurator().apply(ccConfig);
    if (! status.isOK() ) {
      System.out.println("Failed to apply INNER_CANcoder configs. Error code: "+status.toString());
    }
  }

  private void configalgaeArmMotor() {
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
      System.out.println("Failed to apply INNER_ARM configs. Error code: "+status.toString());
    }
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
    GoToPosition(AAC.ALGAE_ARM_SOURCE_POSITION);
  }

  public void GoToScorePosition() {
    GoToPosition(AAC.ALGAE_ARM_SCORE_POSITION);
  }

  public void Scorealgae() {
    m_pinServoMotor.setPosition(AAC.PIN_SERVO_POSITION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
