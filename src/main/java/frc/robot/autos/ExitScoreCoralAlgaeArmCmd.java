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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoC;
import frc.robot.Constants.SDC;
//import frc.robot.commands.WaitForMillisecsCmd;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// ExitSpkrBlueLeftToLeftWallCmd will move a short distance, hugging the Amp side
// wall to avoid disturbing the pre-staged note, to clear the starting zone. 
// This Autononmous move trajectory depends on absolute Y coordinates.

public class ExitScoreCoralAlgaeArmCmd extends SequentialCommandGroup {
  Trajectory exitScoreCoralAlgaeArm;
  SwerveControllerCommand exitScoreCoralAlgaeArmCmd;

  /* Constructor */
  public ExitScoreCoralAlgaeArmCmd(SwerveSubsystem swerveDrive, AlgaeArmSubsystem algaeArmSubsystem, ElevatorSubsystem elevatorSubsystem) {
    TrajectoryConfig exitConfig = new TrajectoryConfig((AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                                        AutoC.AUTO_SPEED_FACTOR_GENERIC),
                                                       (AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                                        AutoC.AUTO_ACCEL_FACTOR_GENERIC))
                                                       .setKinematics(SDC.SWERVE_KINEMATICS);
    exitConfig.setReversed(false);

    exitScoreCoralAlgaeArm = TrajectoryGenerator.generateTrajectory
                                 (
                                 new Pose2d(Units.inchesToMeters(0.0), 
                                            Units.inchesToMeters(0.0), 
                                            Rotation2d.fromDegrees(-180.0)),
                                  List.of(new Translation2d(Units.inchesToMeters(-22.0), 
                                                            Units.inchesToMeters(0.0)),
                                          new Translation2d(Units.inchesToMeters(-44.0),
                                                            Units.inchesToMeters(0)),
                                          new Translation2d(Units.inchesToMeters(-66.0),
                                                            Units.inchesToMeters(0.0))),
                                  new Pose2d(Units.inchesToMeters(-91.0),
                                             Units.inchesToMeters(0.0),
                                             Rotation2d.fromDegrees(-180.0)),
                                  exitConfig
                                 );
  
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoC.KP_THETA_CONTROLLER,
                                                                      AutoC.KI_THETA_CONTROLLER,
                                                                      0,
                                                                      AutoC.K_THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    exitScoreCoralAlgaeArmCmd = new SwerveControllerCommand(exitScoreCoralAlgaeArm,
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
                new InstantCommand(()->algaeArmSubsystem.GoToCoralCradlePosition()),
                new WaitCommand(0.5),
                new InstantCommand(()->swerveDrive.resetOdometry(exitScoreCoralAlgaeArm.getInitialPose())),
                exitScoreCoralAlgaeArmCmd,
                new InstantCommand(()->swerveDrive.stop()),
                new WaitCommand(2),
                new InstantCommand(()->elevatorSubsystem.GoToAutoAlgaePosition()),
                new WaitCommand(0.5),
                new InstantCommand(()->algaeArmSubsystem.GoToScoreCoralPosition()),
                new WaitCommand(2),
                new InstantCommand(()->elevatorSubsystem.GoToL2CoralPosition()),
                new InstantCommand(()->algaeArmSubsystem.GoToL2RemovePosition()),
                new InstantCommand(()->algaeArmSubsystem.RemoveAlgae()),
                new WaitCommand(1),
                new InstantCommand(()->algaeArmSubsystem.StopWheels())
               );
  }
}