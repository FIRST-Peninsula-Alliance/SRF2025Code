// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.NotableConstants.CC;

public class ClimbSubsystem extends SubsystemBase {
  private TalonFX m_climbMotor = new TalonFX(CC.CLIMB_FALCON_ID, Constants.CANIVORE_BUS_NAME);
  private static double m_climbMotorFactor;
  private double m_elevatorPosition;
  private DutyCycleOut m_climbRequest = new DutyCycleOut(CC.CLIMBER_DUTY_CYCLE)
                                                            .withEnableFOC(true)
                                                            .withUpdateFreqHz(50); 
  private CANSparkMax m_elevatorMotor = new CANSparkMax(CC.ELEVATOR_NEO550_ID, MotorType.kBrushless);
  //private SparkPIDController m_elevatorController;
  RelativeEncoder m_integratedElevatorEncoder = m_elevatorMotor.getEncoder();
  private DigitalInput m_climberLimitSwitch = new DigitalInput(1);
  
  private static double m_elevatorFactor;
  private static boolean m_elevatorHasBeenDeployed;
  private static long m_climbStartTime;

  private static boolean m_safetyInterlockRemoved;
  
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    //m_elevatorController = m_elevatorMotor.getPIDController();
    configElevatorMotor();
    m_elevatorPosition = 0.0;
    configClimbMotor();
    m_elevatorFactor = 0.0 * m_elevatorPosition;  // The second arguemnt in the mult here is just to kill a compiler warning of unused variable
    m_climbMotorFactor = 0.0;
    m_elevatorHasBeenDeployed = false;
    m_safetyInterlockRemoved = false;
  }

/****************************************************************
 * Methods to start and stop the elevator. Called from simple
 * buttons, set to 1 or -1 on true edge, set to 0 on false edge.
 * This prevents moving in more then one direction without using
 * normal command "requirements".
 ***************************************************************/
  public void raiseElevator() {
    SmartDashboard.putNumber("MatchTime", Timer.getMatchTime());
    if ((Timer.getMatchTime() < CC.SAFETY_THRESHOLD_TIME_BEFORE_MATCH_END) 
        ||
        m_safetyInterlockRemoved) {
      m_elevatorFactor = 1.0;
      m_elevatorHasBeenDeployed = true;
    }
  }

  public void lowerElevator() {
    m_elevatorFactor = -1.0;
  }

  public void stopElevator() {
    m_elevatorFactor = 0.0;
  }

  /*************************************************************
   * Methods to start and stop the climbing winch,
   * Bound to .onTrue and .onFalse states of a single button
   *************************************************************/
  public void runClimbWinch() {     // single speed, single direction
    if (m_elevatorHasBeenDeployed && 
        ((Timer.getMatchTime() < CC.SAFETY_THRESHOLD_TIME_BEFORE_MATCH_END) 
        ||
        m_safetyInterlockRemoved)) {
      m_climbMotorFactor = 1.0;
      m_climbStartTime = System.currentTimeMillis();
    } else {
      System.out.println("Run winch ignored: either no elevator or too soon");
    }
  }

  public void stopClimbWinch() {
    m_climbMotorFactor = 0.0;
    long climbTime = System.currentTimeMillis() - m_climbStartTime;
    SmartDashboard.putNumber("Climb Time (ms) ", climbTime);
  }

  /************************************************************
   * Method to allow climbing before "end of match" for testing
   ************************************************************/
  public void overrideEndOfMatchSafety() {
    m_safetyInterlockRemoved = true;
    m_elevatorHasBeenDeployed = true;
  }

  /************************************************************
   * During periodic, "run" the motors continuously (they will
   * remain dormant until their respective factors are non zero).
   * Also monitor the motors for overcurrent and stop them 
   * immediately if detected
   ************************************************************/
  @Override
  public void periodic() {
    // Stop elevator and climb motors if current spikes detected. Operator can
    // restart by pressing button(s) again. However, this also risks
    // immediate stops due to inrush current. Use the
    // motor encoder as a safety stop sensor (for both raising and lowering).
   // if ((System.currentTimeMillis() - m_elevatorStartTime) > CC.ELEVATOR_INRUSH_LOCKOUT_TIME) {
   //   if (m_elevatorMotor.getOutputCurrent() > CC.ELEVATOR_SAFETY_THRESHOLD_CURRENT_LIMIT) {
   //     m_elevatorFactor = 0.0;
   //   }
   // }
    if ((System.currentTimeMillis() - m_climbStartTime) > CC.CLIMB_WINCH_INRUSH_LOCKOUT_TIME) {
      if (m_climbMotor.getSupplyCurrent().getValueAsDouble() > CC.WINCH_SAFETY_THRESHOLD_CURRENT_LIMIT) {
        m_climbMotorFactor = 0.0;
      }
    }

    if (m_elevatorFactor > 0.0) {
      m_elevatorPosition = m_integratedElevatorEncoder.getPosition();
      if (m_elevatorPosition >= CC.ELEVATOR_MAX_POS) {
        m_elevatorFactor = 0.0;
        m_elevatorMotor.stopMotor();
        System.out.println("SW Upper Limit tripped on elevator");
      }
    } else if (m_elevatorFactor < 0.0) {
      if (! m_safetyInterlockRemoved) {
        if (m_elevatorPosition <= -5) {
          m_elevatorFactor = 0.0;
          m_elevatorMotor.stopMotor();
          System.out.println("SW Lower Limit tripped on elevator");
        }
      }
    }
    
    m_elevatorMotor.set(CC.ELEVATOR_DUTY_CYCLE *
                        m_elevatorFactor * 
                        SwerveSubsystem.getVarMaxOutputFactor());

    if (m_climbMotorFactor > 0.0) {
      if (! m_climberLimitSwitch.get()) {   // Limit switch is FALSE when triggered
        m_climbMotorFactor = 0.0;
      }
    } else {
      m_climbMotorFactor = 0.0;
    }

    m_climbMotor.setControl(m_climbRequest.withOutput(CC.CLIMBER_DUTY_CYCLE * 
                                                      m_climbMotorFactor )
                                                      .withEnableFOC(true)
                                                      .withUpdateFreqHz(50));
    SmartDashboard.putNumber("Elevator Pos", m_elevatorMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator Amps", m_elevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("Climb Winch Amps", m_climbMotor.getSupplyCurrent().getValueAsDouble());
  }

  public void configElevatorMotor() {
    // restore factory defaults, then set brake mode and if needed, invert rotation direction
    // Motor will be run in simple duty cycle mode at a fixed speed, but will
    // kill speed if current spikes.
    // Note: Operator can easily re-start motor by pressing assigned button(s) again.
    m_elevatorMotor.restoreFactoryDefaults();
    m_elevatorMotor.setIdleMode(IdleMode.kBrake);
    m_elevatorMotor.setInverted(false);
  }
  
  public void configClimbMotor() {
    var climbOutputConfig = new MotorOutputConfigs().withNeutralMode(CC.CLIMB_MOTOR_NEUTRAL_MODE)
                                                     .withInverted(CC.CLIMB_MOTOR_INVERT)
                                                     .withPeakForwardDutyCycle(CC.CLIMB_OUTPUT_LIMIT_FACTOR)
                                                     .withPeakReverseDutyCycle(-CC.CLIMB_OUTPUT_LIMIT_FACTOR);
    var openLoopRampConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(CC.CLIMB_OPEN_LOOP_RAMP_PERIOD)
                                                       .withVoltageOpenLoopRampPeriod(CC.CLIMB_OPEN_LOOP_RAMP_PERIOD);
                                                       //.withTorqueOpenLoopRampPeriod(0);
    var currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(CC.CLIMB_SUPPLY_CURRENT_LIMIT)
                                                       .withSupplyCurrentThreshold(CC.CLIMB_SUPPLY_CURRENT_THRESHOLD)
                                                       .withSupplyTimeThreshold(CC.CLIMB_SUPPLY_CURRENT_TIME_THRESHOLD)
                                                       .withSupplyCurrentLimitEnable(CC.CLIMB_ENABLE_SUPPLY_CURRENT_LIMIT)
                                                       .withStatorCurrentLimit(CC.CLIMB_STATOR_CURRENT_LIMIT)
                                                       .withStatorCurrentLimitEnable(CC.CLIMB_ENABLE_STATOR_CURRENT_LIMIT);
    var climbConfig = new TalonFXConfiguration().withMotorOutput(climbOutputConfig)
                                                .withCurrentLimits(currentLimitConfig)
                                                .withOpenLoopRamps(openLoopRampConfig);
    StatusCode status = m_climbMotor.getConfigurator().apply(climbConfig);

    if (! status.isOK() ) {
        System.out.println("Failed to apply Climb Motor configs. Error code: "+status.toString());
    }
  }
}
