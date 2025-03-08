// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoC;
import frc.robot.Constants.SDC;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// ExitSpkrBlueLeftToLeftWallCmd will move a short distance, hugging the Amp side
// wall to avoid disturbing the pre-staged note, to clear the starting zone. 
// This Autononmous move trajectory depends on absolute Y coordinates.

public class ExitLeftBargeScoreCoralL1 extends SequentialCommandGroup {
  Trajectory exitLeftBargeScoreCoralL1;
  SwerveControllerCommand exitLeftBargeScoreCoralL1Cmd;

  /* Constructor 
  public ExitLeftBargeScoreCoralL1(SwerveSubsystem swerveDrive, CoralArmSubsystem m_coralArmSubsystem) {
    TrajectoryConfig exitLeftBargeConfig = new TrajectoryConfig((AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                                        AutoC.AUTO_SPEED_FACTOR_GENERIC),
                                                       (AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                                        AutoC.AUTO_ACCEL_FACTOR_GENERIC))
                                                       .setKinematics(SDC.SWERVE_KINEMATICS);
    exitLeftBargeConfig.setReversed(false);

    exitLeftBargeScoreCoralL1 = TrajectoryGenerator.generateTrajectory
                                 (
                                 new Pose2d(Units.inchesToMeters(0.0), 
                                            Units.inchesToMeters(0.0), 
                                            Rotation2d.fromDegrees(0.0)),
                                  List.of(new Translation2d(Units.inchesToMeters(-29.0), 
                                                            Units.inchesToMeters(1.5)),
                                          new Translation2d(Units.inchesToMeters(-56.0),
                                                            Units.inchesToMeters(3)),
                                          new Translation2d(Units.inchesToMeters(-95.0),
                                                            Units.inchesToMeters(4.5))),
                                  new Pose2d(Units.inchesToMeters(-112.0),
                                             Units.inchesToMeters(9.0),
                                             Rotation2d.fromDegrees(-60.0)),
                                  exitLeftBargeConfig
                                 );
  
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoC.KP_THETA_CONTROLLER,
                                                                      AutoC.KI_THETA_CONTROLLER,
                                                                      0,
                                                                      AutoC.K_THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    exitLeftBargeScoreCoralL1Cmd = new SwerveControllerCommand(exitLeftBargeScoreCoralL1,
                                                                swerveDrive::getPose,
                                                                SDC.SWERVE_KINEMATICS,
                                                                new PIDController(AutoC.KP_X_CONTROLLER, 
                                                                                  AutoC.KI_X_CONTROLLER,
                                                                                  0),
                                                                new PIDController(AutoC.KP_Y_CONTROLLER, 
                                                                                  AutoC.KI_Y_CONTROLLER, 
                                                                                  0),
                                                                thetaController,
                                                                swerveDrive::setModuleStates,
                                                                swerveDrive);
    addCommands(
                new InstantCommand(()->swerveDrive.resetOdometry(exitLeftBargeScoreCoralL1.getInitialPose())),
                exitLeftBargeScoreCoralL1Cmd,
                new InstantCommand(()->swerveDrive.stop()),
                new InstantCommand(()->m_coralArmSubsystem.GoToScorePosition()),
                new WaitCommand(1.0),
                new InstantCommand(()->m_coralArmSubsystem.ScoreCoral())
               );
  }
} */