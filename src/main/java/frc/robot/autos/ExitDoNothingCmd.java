// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.Constants.AutoC;
import frc.robot.Constants.SDC;
import frc.robot.subsystems.SwerveSubsystem;

// ExitSpkrBlueLeftToLeftWallCmd will move a short distance, hugging the Amp side
// wall to avoid disturbing the pre-staged note, to clear the starting zone. 
// This Autononmous move trajectory depends on absolute Y coordinates.

public class ExitDoNothingCmd extends SequentialCommandGroup {
  Trajectory exitDoNothing;
  SwerveControllerCommand exitDoNothingCmd;

  /* Constructor */
  public ExitDoNothingCmd(SwerveSubsystem swerveDrive) {
    TrajectoryConfig exitConfig = new TrajectoryConfig((AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                                        AutoC.AUTO_SPEED_FACTOR_GENERIC),
                                                       (AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                                        AutoC.AUTO_ACCEL_FACTOR_GENERIC))
                                                       .setKinematics(SDC.SWERVE_KINEMATICS);
    exitConfig.setReversed(false);

    exitDoNothing = TrajectoryGenerator.generateTrajectory
                                 (
                                 new Pose2d(Units.inchesToMeters(0.0), 
                                            Units.inchesToMeters(0.0), 
                                            Rotation2d.fromDegrees(0.0)),
                                  List.of(new Translation2d(Units.inchesToMeters(-12.0), 
                                                            Units.inchesToMeters(0.0)),
                                          new Translation2d(Units.inchesToMeters(-24.0),
                                                            Units.inchesToMeters(0)),
                                          new Translation2d(Units.inchesToMeters(-36.0),
                                                            Units.inchesToMeters(0.0))),
                                  new Pose2d(Units.inchesToMeters(-48.0),
                                             Units.inchesToMeters(0.0),
                                             Rotation2d.fromDegrees(-90.0)),
                                  exitConfig
                                 );
  
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoC.KP_THETA_CONTROLLER,
                                                                      AutoC.KI_THETA_CONTROLLER,
                                                                      0,
                                                                      AutoC.K_THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    exitDoNothingCmd = new SwerveControllerCommand(exitDoNothing,
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
                new InstantCommand(()->swerveDrive.resetOdometry(exitDoNothing.getInitialPose())),
                exitDoNothingCmd,
                new InstantCommand(()->swerveDrive.stop())
               );
  }
}