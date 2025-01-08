// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.TrajectoryPaths;

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

public class MoveRightNoteToDistantShotCmd extends SequentialCommandGroup {

  /* Constructor */
  public MoveRightNoteToDistantShotCmd(SwerveSubsystem swerveDrive) {
    TrajectoryConfig moveConfig = new TrajectoryConfig((AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                                        AutoC.AUTO_SPEED_FACTOR_GENERIC),
                                                       (AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                                        AutoC.AUTO_ACCEL_FACTOR_GENERIC))
                                                       .setKinematics(SDC.SWERVE_KINEMATICS);
    moveConfig.setReversed(true);

    Trajectory moveRightNoteToDistantShot =
      TrajectoryGenerator.generateTrajectory(
          // Start at the end Pose2d of the MoveDistantShotToRightNote trajectory
          new Pose2d(Units.inchesToMeters(153.0), 
                      Units.inchesToMeters(5.0), 
                      Rotation2d.fromDegrees(275.0)),
          List.of(new Translation2d(Units.inchesToMeters(152.0), 
                                    Units.inchesToMeters(6.0)),
                  new Translation2d(Units.inchesToMeters(130.0), 
                                    Units.inchesToMeters(26.0)),
                  new Translation2d(Units.inchesToMeters(113.0), 
                                    Units.inchesToMeters(56.0))),
          new Pose2d(Units.inchesToMeters(112.0),
                      Units.inchesToMeters(57.0),
                      Rotation2d.fromDegrees(0.0)),
          moveConfig);

    ProfiledPIDController thetaController = new ProfiledPIDController(AutoC.KP_THETA_CONTROLLER,
                                                                      AutoC.KI_THETA_CONTROLLER,
                                                                      0,
                                                                      AutoC.K_THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand moveRightNoteToDistantShotCmd =
                                  new SwerveControllerCommand(moveRightNoteToDistantShot,
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
                moveRightNoteToDistantShotCmd,
                new InstantCommand(()->swerveDrive.stop())
               );
  }
}
