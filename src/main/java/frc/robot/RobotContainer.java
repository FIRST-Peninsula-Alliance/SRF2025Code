package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.NotableConstants.SC;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeArmSubsystem;
//import frc.robot.subsystems.CoralArmSubsystem;
//import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Subsystem local object handles */
    private SwerveSubsystem          m_swerveSubsystem;
    // private MasterArmSubsystem       m_masterArmSubsystem;
    // private ClimbSubsystem           m_climbSubsystem;
    private ElevatorSubsystem       m_elevatorSubsytem;
    private ClimberSubsystem        m_climberSubsystem;
    //private CoralArmSubsystem       m_coralArmSubsystem;
    private AlgaeArmSubsystem       m_algaeArmSubsystem;
    //private VisionSubsystem         m_visionSubsystem;

    private SwerveParkCmd            m_parkCmd;

    // Declare handles for choosable autonomous Commands
    //private TestSquareAuto           m_test_square_auto;

    // Create sendable choosers for starting position and desired Auto routine
    private static SendableChooser<Command> m_autoRoutineChooser = new SendableChooser<>();

    // Declare CommandXboxController
    private static CommandXboxController m_xbox;

    //  Constructor for the robot container. Contains subsystems, OI devices, and commands.
    public RobotContainer() {
        m_xbox = new CommandXboxController(0);

        m_swerveSubsystem = new SwerveSubsystem();
        // m_masterArmSubsystem = new MasterArmSubsystem();
        // m_climbSubsystem = new ClimbSubsystem();
        m_elevatorSubsytem = new ElevatorSubsystem();
        m_climberSubsystem = new ClimberSubsystem();
        //m_coralArmSubsystem = new CoralArmSubsystem();
        m_algaeArmSubsystem = new AlgaeArmSubsystem();
        // m_visionSubsystem = new VisionSubsystem();

        m_swerveSubsystem.setDefaultCommand(
                new DefaultDriveCmd(m_swerveSubsystem,
                // Xbox stick forward and stick to left are both neg, so must negate all for WPILib expected coordinates:
                                    () -> -m_xbox.getLeftY(),    // translate: +fore / -back
                                    () -> -m_xbox.getLeftX(),    // strafe: +left / -right
                                    () -> -m_xbox.getRightX())); // rotate: +CCW / -CW

        m_parkCmd = new SwerveParkCmd(m_swerveSubsystem,
                                      () -> -m_xbox.getLeftY(),
                                      () -> -m_xbox.getLeftX(),
                                      () -> -m_xbox.getRightX());
                    
        ExitDoNothingCmd m_exitDoNothingAuto = new ExitDoNothingCmd(m_swerveSubsystem);
        ExitScoreCoralAlgaeArmCmd m_ExitScoreCoralAlgaeArmCmd = new ExitScoreCoralAlgaeArmCmd(m_swerveSubsystem, m_algaeArmSubsystem, m_elevatorSubsytem);
        //ExitLeftBargeScoreCoralL1 m_exitLeftBargeScoreCoralL1 = new ExitLeftBargeScoreCoralL1(m_swerveSubsystem, m_coralArmSubsystem);
        //ExitRightBargeScoreCoralL1 m_exitRightBargeScoreCoralL1 = new ExitRightBargeScoreCoralL1(m_swerveSubsystem, m_coralArmSubsystem);

        m_autoRoutineChooser.setDefaultOption("ExitDoNothing", m_exitDoNothingAuto);
        m_autoRoutineChooser.addOption("ExitScoreCoralAlgaeArm", m_ExitScoreCoralAlgaeArmCmd);
        //m_autoRoutineChooser.addOption("ExitLeftBargeScoreCoralL1", m_exitLeftBargeScoreCoralL1);
        //m_autoRoutineChooser.addOption("ExitRightBargeScoreCoralL1", m_exitRightBargeScoreCoralL1);
        SmartDashboard.putData("Autonomous Selection:", m_autoRoutineChooser);

        configureButtonBindings();
    }
    
    /**************************************************************
     * Getters for all subsystem Classes and other useful objects
     **************************************************************/
    public static XboxController getHidXboxCtrl(){
        return m_xbox.getHID();
    }

    public void respondToBeingDisabled() {
        // m_masterArmSubsystem.stopWavingAtCrowd();
        // m_masterArmSubsystem.closeRecording();
    }

    /***********************************************
     * Button Bindings defines the operator UI
     ***********************************************/
    private void configureButtonBindings() {
        final Trigger ALT = m_xbox.leftBumper();

        // The following govern the UI for the Cresendo Season:
        
        
        //    Left Bumbper          => ALT mode for oter buttons when held. No action on its own
        //    Right Bumper          => Slow mode when held
        //    ALT + Right bumper    => super slow when held
        //    a                     => Elevator L1 Position
        //    b                     => Elevator L2 Position
        //    y                     => Elevator L3 Position
        //    ALT + a               => Algae Arm Up Position
        //    ALT + b               => Algae Arm Ground Pickup Position
        //    ALT + y               => Algae Arm Remove Position
        //    RightTrigger          => Coral Arm Score Position
        //    ALT + RightTrigger    => Release Coral
        //    LeftTrigger           => Coral Arm Source Position
        //    ALT + LeftTrigger     => Coral Arm Center Position
        //    x                     => Run Algae Wheels in Remove Direction
        //    ALT + x               => Park (crossed wheel angles)
        //    L Joystick Y          => Swerve Translate (move fore/aft)
        //    L Joystick X          => Swerve Strafe (move side to side)
        //    R Joystick X          => Swerve Rotate (left = CCW, right = CW)
        //    R Joystick Y          => No Use
        //    L Joystick Button     => Set Field Oriented drive
        //    R Joystick Button     => Set Robot Oriented drive
        //    Back                  => Zero the Gyro
        //    Start                 => No Use
        //    ALT + Start           => No Use
        //    ALT + Back            => No Use
        //    POV_UP                => Run Algae Wheels in Process Direction
        //    POV_DOWN              => Run Algae Wheels in Pickup Direction
        //    POV_LEFT              => No Use
        //    POV_RIGHT             => No Use
        //    ALT + POV_UP          => No Use
        //    ALT + POV_DOWN        => No Use
        //    ALT + POV_LEFT        => No Use
        //    ALT + POV_RIGHT       => No Use

        /*
        The following methods can be used to trigger rotation about any given corner. Most useful when
        playing defense. Retained here in case a quick change to defense at competition
        is needed, but normally these button bindings are needed for offense.
        m_xbox.leftTrigger().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFLCenOfRotation()));
        m_xbox.leftTrigger().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        m_xbox.rightTrigger().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFRCenOfRotation()));                                        
        m_xbox.rightTrigger().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));  
        m_xbox.leftBumper().onTrue(new InstantCommand(()-> m_swerveSubsystem.setBLCenOfRotation()));
        m_xbox.leftBumper().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        m_xbox.rightBumper().onTrue(new InstantCommand(()-> m_swerveSubsystem.setBRCenOfRotation()));
        m_xbox.rightBumper().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        */

        // left and right trigger set coral arm positions 
        // alt-right trigger score coral
        //m_xbox.leftTrigger().onTrue(new InstantCommand(() -> m_coralArmSubsystem.GoToSourcePosition()));
        //m_xbox.rightTrigger().onTrue(new InstantCommand(() -> m_coralArmSubsystem.ScoreL1Position()));
        //m_xbox.rightTrigger().and(ALT).onTrue(new InstantCommand(() -> m_coralArmSubsystem.ScoreCoral()));
        // center coral arm position
        //m_xbox.leftTrigger().and(ALT).onTrue(new InstantCommand(() -> m_coralArmSubsystem.GoToCenterPosition()));

        // Left and right joystick buttons determine field oriented or robot oriented driving
        m_xbox.leftStick().and(ALT.negate()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setFieldOriented(true)));
        m_xbox.rightStick().and(ALT.negate()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setFieldOriented(false)));
        // Zero Gyro
        m_xbox.back().and(ALT.negate()).onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));   // was resetModulesToAbsolute()));

        // Right bumper alone = slow mode.
        // Alt + Right Bumper = very slow mode.
        // On Right bumper release (regardless of Left Bumper state), full speed.
        m_xbox.rightBumper().and(ALT.negate()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(.3)));
        ALT.and(m_xbox.rightBumper()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(.1)));
        m_xbox.rightBumper().onFalse(new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(1.0)));;
        
        // Move Elevator Positions
        m_xbox.a().and(ALT.negate()).onTrue(new InstantCommand(()-> m_elevatorSubsytem.GoToL1CoralPosition()));
        m_xbox.b().and(ALT.negate()).onTrue(new InstantCommand(()-> m_elevatorSubsytem.GoToL2CoralPosition()));
        m_xbox.y().and(ALT.negate()).onTrue(new InstantCommand(()-> m_elevatorSubsytem.GoToL3CoralPosition()));

        // Alage Arm Positions
        m_xbox.a().and(ALT).onTrue(new InstantCommand(()-> m_algaeArmSubsystem.GoToUpPosition()));
        m_xbox.b().and(ALT).onTrue(new InstantCommand(()-> m_algaeArmSubsystem.GoToPickupPosition()));
        m_xbox.y().and(ALT).onTrue(new InstantCommand(()-> m_algaeArmSubsystem.GoToL2RemovePosition()));

        // Algae Wheel Bindings
        m_xbox.x().and(ALT.negate()).onTrue(new InstantCommand(()-> m_algaeArmSubsystem.RemoveAlgae()));
        m_xbox.x().and(ALT.negate()).onFalse(new InstantCommand(()-> m_algaeArmSubsystem.StopWheels()));
        m_xbox.povDown().and(ALT.negate()).onTrue(new InstantCommand(()-> m_algaeArmSubsystem.PickupAlgae()));
        m_xbox.povUp().and(ALT.negate()).onTrue(new InstantCommand(()-> m_algaeArmSubsystem.ScoreAlgae()));
        m_xbox.povUp().and(ALT.negate()).onFalse(new InstantCommand(()-> m_algaeArmSubsystem.StopWheels()));

        // Climber Bindings
        m_xbox.rightTrigger().and(ALT.negate()).onTrue(new InstantCommand(()-> m_climberSubsystem.runWinch()));
        m_xbox.rightTrigger().and(ALT).onTrue(new InstantCommand(()-> m_climberSubsystem.StopWinch()));
        m_xbox.leftTrigger().onTrue(new InstantCommand(()-> m_climberSubsystem.DisengageSpringServo()));
        m_xbox.leftTrigger().onTrue(new InstantCommand(()-> m_climberSubsystem.ReleaseClimberPin()));

        m_xbox.povUp().and(ALT).onTrue(new InstantCommand(()-> m_climberSubsystem.IncrementSpringServo()));
        m_xbox.povRight().and(ALT).onTrue(new InstantCommand(()-> m_climberSubsystem.IncrementLinearServo()));
        m_xbox.povLeft().and(ALT).onTrue(new InstantCommand(()-> m_climberSubsystem.DecrementLinearServo()));
        m_xbox.povDown().and(ALT).onTrue(new InstantCommand(()-> m_climberSubsystem.DecrementSpringServo()));

        // m_xbox.x().and(ALT.negate()).onTrue(new InstantCommand(()->m_masterArmSubsystem.cancelNoteAction()));
        // Swerve park 
        ALT.and(m_xbox.x()).onTrue(m_parkCmd);
        
        // Note handling activities
        /* m_xbox.b().onTrue(new InstantCommand(()->m_masterArmSubsystem.acquireNote()));
        m_xbox.a().and(ALT.negate()).onTrue(new InstantCommand(()->m_masterArmSubsystem.prepForAmpScore()));
        ALT.and(m_xbox.a()).onTrue(new InstantCommand(()->m_masterArmSubsystem.prepForDistantSpeakerScore(SC.SHOOTER_VOLTAGE_OUT_PASS)));
        m_xbox.y().and(ALT.negate()).onTrue(new InstantCommand(()->m_masterArmSubsystem.prepForIndexedSpeakerScore()));
        ALT.and(m_xbox.y()).onTrue(new InstantCommand(()->m_masterArmSubsystem.prepForDistantSpeakerScore(SC.SHOOTER_VOLTAGE_OUT_FAR)));
        ALT.and(m_xbox.rightTrigger(0.5)).onTrue(new InstantCommand(()->m_masterArmSubsystem.discardNote()));
        m_xbox.rightTrigger(0.5).and(ALT.negate()).onTrue(new InstantCommand(()->m_masterArmSubsystem.scoreNote()));
        m_xbox.start().and(ALT.negate()).onTrue(new InstantCommand(()->m_masterArmSubsystem.stepPastDebugHold()));
        ALT.and(m_xbox.start()).onTrue(new WaveCmd(m_swerveSubsystem, m_masterArmSubsystem));
        m_xbox.leftTrigger().and(ALT.negate()).onTrue(new InstantCommand(()->m_masterArmSubsystem.simulateNoteAcquired())); */
    
        // Utilities for fine tuning various arm positions, can be used in extremis
        // during a match if pickup angles are not working.
        // Use povRight, povLeft, povUp and povDown for fine setpoint adjustments, 
        // about 1 degree per button press)
    /*
        m_xbox.povLeft().and(ALT.negate()).onTrue(new InstantCommand(()-> m_masterArmSubsystem.adjustInnerArmSetpointUp()));
        m_xbox.povRight().and(ALT.negate()).onTrue(new InstantCommand(()-> m_masterArmSubsystem.adjustInnerArmSetpointDown()));
        m_xbox.povUp().and(ALT.negate()).onTrue(new InstantCommand(()-> m_masterArmSubsystem.adjustMasterArmSetpointUp()));
        m_xbox.povDown().and(ALT.negate()).onTrue(new InstantCommand(()-> m_masterArmSubsystem.adjustMasterArmSetpointDown()));

        // climb activities all require ALT button combination
        ALT.and(m_xbox.back()).onTrue(new InstantCommand(()->m_climbSubsystem.overrideEndOfMatchSafety()));
        ALT.and(m_xbox.povUp()).onTrue(new InstantCommand(()-> m_climbSubsystem.raiseElevator()));
        ALT.and(m_xbox.povUp()).onFalse(new InstantCommand(()-> m_climbSubsystem.stopElevator()));
        ALT.and(m_xbox.povDown()).onTrue(new InstantCommand(()-> m_climbSubsystem.lowerElevator()));
        ALT.and(m_xbox.povDown()).onFalse(new InstantCommand(()-> m_climbSubsystem.stopElevator()));
        ALT.and(m_xbox.leftTrigger()).onTrue(new InstantCommand(()-> m_climbSubsystem.runClimbWinch()));
        ALT.and(m_xbox.leftTrigger()).onFalse(new InstantCommand(()-> m_climbSubsystem.stopClimbWinch()));
    */
    }

    /*
     * getSelectedAutoCommand is called from Robot.AutonomousInit(),
     * so if a good place to also check which position was selected for
     * the start of match, and if left or right is selected, then we can 
     * initialize the gyro to either 60.0 (for left), or 300.0 degrees
     * (for right).
     * The default is for the gyro to boot to 0 degrees, so no need to cover
     * any case other than left or right start position for setting the gyro.
     */
    public Command getSelectedAutoCommand() {
        Command selectedAuto = m_autoRoutineChooser.getSelected();

        if (selectedAuto == null) {
            selectedAuto = new DoNothingCmd();
        }
        return selectedAuto;
    }

    public void teleopStart() {
        // m_masterArmSubsystem.teleopStart();
    }
}