package frc.robot;

import java.text.DecimalFormat;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
// import com.ctre.phoenix6.configs.MagnetSensorConfigs.AbsoluteSensorDiscontinuityPoint;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.units.measure.Current;
//import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.swerve.SDS_SwerveUnitParams;
import frc.lib.swerve.SwerveModuleConstants;

public final class Constants {
    /***************************************************
     * Universal Constants
     ***************************************************/
    public static final String CANIVORE_BUS_NAME = "CANivore";
    public static final String ROBO_RIO_BUS_NAME = "rio";
    public static final String CAN_BUS_IN_USE = "CANivore";

    public static final class F {
        // Formatters to control number of decimal places 
        // in the various published / recorded data
        public static DecimalFormat df1 = new DecimalFormat("#.#");
        public static DecimalFormat df2 = new DecimalFormat("#.##");
        public static DecimalFormat df3 = new DecimalFormat("#.###");
        public static DecimalFormat df4 = new DecimalFormat("#.####");
        public static DecimalFormat df20 = new DecimalFormat("##");
        public static DecimalFormat df40 = new DecimalFormat("####");
        public static DecimalFormat df80 = new DecimalFormat("########");
    }

    /****************************************************
     * User Interface Constants
     ****************************************************/
    public static final class UIC {             // UIC = short for UserInterfaceConstants
        public static final double JOYSTICK_DEADBAND = 0.18;
    }

    /*****************************************************
     * Gyro Constants
     *****************************************************/
    public static final class GC {              // GC = short for GyroConstants
        public static final int PIGEON_2_CANID = 1;
        public static final boolean INVERT_GYRO = false; // In phoenix6, Pigeon2.getAngle() api
                                                        // returns heading, but it increases with
                                                        // CW rotation, the opposite of WPILib
                                                        // convention, which is CCW+ CW-,
                                                        // So if using getAngle(), set this to true.
                                                        // However, getRotation2d() instead can be used
                                                        // to return angles suited to WPILib
                                                        // conventions, with no gyro inversion.

        // if NavX, use NavX library to init (no need for an ID), but be sure to
        // set INVERT_GYRO = true; // because NavX measures CCW as negative.
    }

    /******************************************************
     * Swerve Drive Constants
     ******************************************************/
    public static final class SDC {             // SDC = short for SwerveDriveConstants

        public static final double BILLET_WHEEL_DIA_INCHES = 4.0;
        public static final double COLSON_WHEEL_DIA_INCHES = 3.96;
        public static final double PARADE_WHEEL_DIA_INCHES = 3.75;

        // The 2025 chassis (Crush) measures 28" x 32", and uses Mk4i-L2 modules.
        // The wheel to wheel distances for Crush are: Width = 22.75", and length = 26.75"
        // (wheels are inset just 2-5/8" from frame sides).
        // The 2024 chassis (Tsunami) measures 28" x 30", and uses Mk4i-L2 modules. 
        // The wheel to wheel disances for Tsunami are: Width = 22.75", and length = 24.75" 
        // (wheels are inset just 2-5/8" from frame sides).
        // The 2023 chassis (BlackKnight) measures 26" x 30" and uses Mk4-L2 modules. 
        // The wheel to wheel distances for BlackKnight are: Width = 19.5", and length = 23.5"
        // (wheels are inset 3.25" from frame sides).
        public static final double BLACK_KNIGHT_TRACK_WIDTH = Units.inchesToMeters(19.5); 
        public static final double BLACK_KNIGHT_WHEEL_BASE = Units.inchesToMeters(23.5); 
        public static final double HALF_BKTW = BLACK_KNIGHT_TRACK_WIDTH / 2.0;
        public static final double HALF_BKWB = BLACK_KNIGHT_WHEEL_BASE / 2.0;

        public static final double TSUNAMI_TRACK_WIDTH = Units.inchesToMeters(22.75); 
        public static final double TSUNAMI_WHEEL_BASE = Units.inchesToMeters(24.75); 
        public static final double HALF_TTW = TSUNAMI_TRACK_WIDTH / 2.0;
        public static final double HALF_TWB = TSUNAMI_WHEEL_BASE / 2.0;

        public static final double CRUSH_TRACK_WIDTH = Units.inchesToMeters(22.75); 
        public static final double CRUSH_WHEEL_BASE = Units.inchesToMeters(26.75); 
        public static final double HALF_CTW = TSUNAMI_TRACK_WIDTH / 2.0;
        public static final double HALF_CWB = TSUNAMI_WHEEL_BASE / 2.0;

        public static final Translation2d BK_FL = new Translation2d(HALF_BKWB, HALF_BKTW);                                    
        public static final Translation2d BK_FR = new Translation2d(HALF_BKWB, -HALF_BKTW);
        public static final Translation2d BK_BL = new Translation2d(-HALF_BKWB, HALF_BKTW);
        public static final Translation2d BK_BR = new Translation2d(-HALF_BKWB, -HALF_BKTW);

        public static final Translation2d T_FL = new Translation2d(HALF_TWB, HALF_TTW);                                    
        public static final Translation2d T_FR = new Translation2d(HALF_TWB, -HALF_TTW);
        public static final Translation2d T_BL = new Translation2d(-HALF_TWB, HALF_TTW);
        public static final Translation2d T_BR = new Translation2d(-HALF_TWB, -HALF_TTW);

        public static final Translation2d C_FL = new Translation2d(HALF_CWB, HALF_CTW);                                    
        public static final Translation2d C_FR = new Translation2d(HALF_CWB, -HALF_CTW);
        public static final Translation2d C_BL = new Translation2d(-HALF_CWB, HALF_CTW);
        public static final Translation2d C_BR = new Translation2d(-HALF_CWB, -HALF_CTW);

        // Offsets for changing the center of rotation, if needed, are the same as the
        // Translation2d coordinates of each corner, plus the center of the robot.
        // See SwerveSubsystem. Corner rotations are especially useful for blocking 
        // while playing defense, and sometimes even for evasion while on offense, but 
        // requires (and ties up) a significant number of GameController button resources
        // (min 4 buttons, or 5 if not using temporary "active only when held" buttons) 
        // so doing this may not be practical for a given season.
        public static final Translation2d   REL_POS2D_CEN = new Translation2d(0, 0);

        // Wheel angles for Back KNight to "park" - makes it difficult to move, slide, or be pushed
        public static final double BK_PARK_ANGLE_LEFT_DEG = 50.2;
        public static final double BK_PARK_ANGLE_RIGHT_DEG = 129.8;

        // Wheel angles for Tsunami "park"
        public static final double T_PARK_ANGLE_LEFT_DEG = 45.0;
        public static final double T_PARK_ANGLE_RIGHT_DEG = -45.0;

        // Wheel angles for Crush "park"
        public static final double C_PARK_ANGLE_LEFT_DEG = 45.0;
        public static final double C_PARK_ANGLE_RIGHT_DEG = -45.0;

        // Differentiate SDS module types MK4 and MK4i
        // which are used with the Black Knight and Tsunami chassis, respectively
        // 
        // Swerve Drive Specialties: MK4-L2 Module on Black Knight
        public static SDS_SwerveUnitParams SDSMK4_BLACK_KNIGHT(double wheelDiaInches){
            double wheelDiaM = Units.inchesToMeters(wheelDiaInches);
            Translation2d relPos2D_FL = BK_FL;
            Translation2d relPos2D_FR = BK_FR;
            Translation2d relPos2D_BL = BK_BL;
            Translation2d relPos2D_BR = BK_BR;
            // TODO: measure and enter BlackKnight absolute wheel angle offsets, in degrees, here
            Rotation2d absOffsetFL = Rotation2d.fromDegrees(250.8);
            Rotation2d absOffsetFR = Rotation2d.fromDegrees(252.4);
            Rotation2d absOffsetBL = Rotation2d.fromDegrees(305.3);
            Rotation2d absOffsetBR = Rotation2d.fromDegrees(103.0);
            double parkAngleLeftDeg = BK_PARK_ANGLE_LEFT_DEG;
            double parkAngleRightDeg = BK_PARK_ANGLE_RIGHT_DEG;
            double steerGearRatio = (12.8 / 1.0);
            // L2 Drive Gear Ratio for both the MK4 and MK4i module is 6.75 
            // and SRF uses only the L2 ratio.
            double driveL2GearRatio = (6.75 / 1.0);
            double steerKP = 0.005;         // Was .2 for angles in radians
            double steerKI = 0.0;
            double steerKD = 0.0;
            double steerKF = 0.0;
            InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;
            InvertedValue steerMotorInvert = InvertedValue.Clockwise_Positive; //TODO: Look here if steering is inverted
            SensorDirectionValue canCoderDir = SensorDirectionValue.CounterClockwise_Positive;
            return new SDS_SwerveUnitParams(wheelDiaM,
                                            relPos2D_FL,
                                            relPos2D_FR,
                                            relPos2D_BL,
                                            relPos2D_BR,
                                            absOffsetFL, 
                                            absOffsetFR, 
                                            absOffsetBL, 
                                            absOffsetBR, 
                                            parkAngleLeftDeg,
                                            parkAngleRightDeg,
                                            steerGearRatio, 
                                            driveL2GearRatio, 
                                            steerKP, 
                                            steerKI, 
                                            steerKD, 
                                            steerKF, 
                                            driveMotorInvert, 
                                            steerMotorInvert, 
                                            canCoderDir);
        }

        // Swerve Drive Specialties: MK4I-L2 Module
        public static SDS_SwerveUnitParams SDSMK4i_TSUNAMI(double wheelDiaInches){
            double wheelDiaM = Units.inchesToMeters(wheelDiaInches);
            Translation2d relPos2D_FL = T_FL;
            Translation2d relPos2D_FR = T_FR;
            Translation2d relPos2D_BL = T_BL;
            Translation2d relPos2D_BR = T_BR;
            // TODO: measure and enter Tsunami absolute wheel angle offsets, in degrees, here
            Rotation2d absOffsetFL = Rotation2d.fromDegrees(121.1);
            Rotation2d absOffsetFR = Rotation2d.fromDegrees(335.41);
            Rotation2d absOffsetBL = Rotation2d.fromDegrees(261.7);
            Rotation2d absOffsetBR = Rotation2d.fromDegrees(35.58);
            double parkAngleLeftDeg = T_PARK_ANGLE_LEFT_DEG;
            double parkAngleRightDeg = T_PARK_ANGLE_RIGHT_DEG;
            double steerGearRatio = ((150.0 / 7.0) / 1.0);
            // L2 Drive Gear Ratio for both the MK4 and MK4i module is 6.75 
            // and SRF only uses the L2 ratio.
            double driveL2GearRatio = (6.75 / 1.0);
            double steerKP = 0.008;             // was .3 for angles in radians
            double steerKI = 0.0;
            double steerKD = 0.0;
            double steerKF = 0.0;
            InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;   // Tsunami is inverted from Black Knight
            InvertedValue steerMotorInvert = InvertedValue.CounterClockwise_Positive; //TODO: Look here if steering is inverted
            SensorDirectionValue canCoderDir = SensorDirectionValue.CounterClockwise_Positive;
            
            return new SDS_SwerveUnitParams(wheelDiaM,
                                            relPos2D_FL,
                                            relPos2D_FR,
                                            relPos2D_BL,
                                            relPos2D_BR,
                                            absOffsetFL,
                                            absOffsetFR,
                                            absOffsetBL,
                                            absOffsetBR,
                                            parkAngleLeftDeg,
                                            parkAngleRightDeg,
                                            steerGearRatio, 
                                            driveL2GearRatio, 
                                            steerKP, 
                                            steerKI, 
                                            steerKD, 
                                            steerKF, 
                                            driveMotorInvert, 
                                            steerMotorInvert, 
                                            canCoderDir);
        }

        public static SDS_SwerveUnitParams SDSMK4i_CRUSH(double wheelDiaInches){
            double wheelDiaM = Units.inchesToMeters(wheelDiaInches);
            Translation2d relPos2D_FL = C_FL;
            Translation2d relPos2D_FR = C_FR;
            Translation2d relPos2D_BL = C_BL;
            Translation2d relPos2D_BR = C_BR;
            // TODO: measure and enter Crush absolute wheel angle offsets, in degrees, here
            Rotation2d absOffsetFL = Rotation2d.fromDegrees(287.8); //was 72.2 deg
            Rotation2d absOffsetFR = Rotation2d.fromDegrees(342.5); //was 219.5 deg
            Rotation2d absOffsetBL = Rotation2d.fromDegrees(82.0);  //was 278.0 deg
            Rotation2d absOffsetBR = Rotation2d.fromDegrees(46.2);  //was 313.8 deg
            double parkAngleLeftDeg = C_PARK_ANGLE_LEFT_DEG;
            double parkAngleRightDeg = C_PARK_ANGLE_RIGHT_DEG;
            double steerGearRatio = ((150.0 / 7.0) / 1.0);
            // L2 Drive Gear Ratio for both the MK4 and MK4i module is 6.75 
            // and SRF only uses the L2 ratio.
            double driveL2GearRatio = (6.75 / 1.0);
            double steerKP = 75;             // was .3 for angles in radians
            double steerKI = 0.0;
            double steerKD = 0.0;
            double steerKF = 0.0;
            InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;   // Crush is inverted from Black Knight
            InvertedValue steerMotorInvert = InvertedValue.Clockwise_Positive; //TODO: Look here if steering is inverted
            SensorDirectionValue canCoderDir = SensorDirectionValue.CounterClockwise_Positive;
            
            return new SDS_SwerveUnitParams(wheelDiaM,
                                            relPos2D_FL,
                                            relPos2D_FR,
                                            relPos2D_BL,
                                            relPos2D_BR,
                                            absOffsetFL,
                                            absOffsetFR,
                                            absOffsetBL,
                                            absOffsetBR,
                                            parkAngleLeftDeg,
                                            parkAngleRightDeg,
                                            steerGearRatio, 
                                            driveL2GearRatio, 
                                            steerKP, 
                                            steerKI, 
                                            steerKD, 
                                            steerKF, 
                                            driveMotorInvert, 
                                            steerMotorInvert, 
                                            canCoderDir);
        }
        
        // Set CHOOSEN_MODULE to either the SDSMK4 or SDSMK4I static method
         public static final SDS_SwerveUnitParams CHOOSEN_MODULE =  
                                                    // Uncomment only one module type,
                                                    // and pass the installed wheel
                                                    // diameter as an argument
                                                    // SDSMK4_BLACK_KNIGHT(PARADE_WHEEL_DIA_INCHES);
                                                    SDSMK4i_CRUSH(COLSON_WHEEL_DIA_INCHES);

        // Now use CHOOSEN_MODULE to initialize generically named (and usable)
        // constants but which are specific to a given module type. 
        // Note some constants are actually the same between the only two SDS module
        // types which SRF currently uses, but this approach, originally designed 
        // to support the declaration of any COTS swerve module, was easier to leave 
        // mostly intact, especially since with this revision it should be much easier 
        // to understand for newbie programmers while still teaching a useful 
        // abstraction technique.
        public static final double WHEEL_DIAMETER_M = CHOOSEN_MODULE.WHEEL_DIAMETER_M;
        public static final double WHEEL_CIRCUMFERENCE_M = CHOOSEN_MODULE.WHEEL_CIRCUMFERENCE_M;
        public static final Translation2d REL_POS2D_FL = CHOOSEN_MODULE.REL_POS2D_FL;
        public static final Translation2d REL_POS2D_FR = CHOOSEN_MODULE.REL_POS2D_FR;
        public static final Translation2d REL_POS2D_BL = CHOOSEN_MODULE.REL_POS2D_BL;
        public static final Translation2d REL_POS2D_BR = CHOOSEN_MODULE.REL_POS2D_BR;
        public static final Rotation2d ABS_OFFSET_FL = CHOOSEN_MODULE.ABS_OFFSET_FL;
        public static final Rotation2d ABS_OFFSET_FR = CHOOSEN_MODULE.ABS_OFFSET_FR;
        public static final Rotation2d ABS_OFFSET_BL = CHOOSEN_MODULE.ABS_OFFSET_BL;
        public static final Rotation2d ABS_OFFSET_BR = CHOOSEN_MODULE.ABS_OFFSET_BR;
        public static final double PARK_ANGLE_LEFT_DEG = CHOOSEN_MODULE.PARK_ANGLE_LEFT_DEG;
        public static final double PARK_ANGLE_RIGHT_DEG = CHOOSEN_MODULE.PARK_ANGLE_LEFT_DEG;
        public static final double DRIVE_GEAR_RATIO = CHOOSEN_MODULE.DRIVE_GEAR_RATIO;
        public static final double STEER_GEAR_RATIO = CHOOSEN_MODULE.STEER_GEAR_RATIO;
        public static final double STEER_KP = CHOOSEN_MODULE.STEER_KP;
        public static final double STEER_KI = CHOOSEN_MODULE.STEER_KI;
        public static final double STEER_KD = CHOOSEN_MODULE.STEER_KD;
        public static final double STEER_KF = CHOOSEN_MODULE.STEER_KF;
        public static final InvertedValue STEER_MOTOR_INVERT = CHOOSEN_MODULE.STEER_MOTOR_INVERT;
        public static final InvertedValue DRIVE_MOTOR_INVERT = CHOOSEN_MODULE.DRIVE_MOTOR_INVERT;
        public static final SensorDirectionValue CANCODER_DIR = CHOOSEN_MODULE.CANCODER_DIR;
        // Set Swerve Kinematics.  The order is always FL, FR, BL, and BR, 
        // referenced to an origin at the center of the robot
        // as evidenced by following the signs of the sequential 
        // Translation2d(x, y) coordinates below:
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = 
                                new SwerveDriveKinematics(REL_POS2D_FL, REL_POS2D_FR, REL_POS2D_BL, REL_POS2D_BR);

        // Swerve Drive Constants which are independent of given modules and chassis:
        public static final int CANCODER_RANGE = 1; //TODO: AbsoluteSensorDiscontinuityPointValue CANCODER_RANGE = AbsoluteSensorDiscontinuityPointValue.Unsigned_0To1;

        // Unit conversion factors. With Phoenix6, the gear ratios are handled by the
        // motor controllers, so that motor.getPosition() values (assuming sensor source is
        // the embedded motor encoder) track the actual mechanism rotations. Thus the 
        // conversion factor for drive system rotations to meters of travel, and for drive
        // rotational velocity (RPS) to linear velopcity (MPS), is just 
        // the driven wheel circumference.
        public static final double TALONFX_ROT_TO_M_FACTOR = WHEEL_CIRCUMFERENCE_M;
        public static final double M_TO_TALONFX_ROT_FACTOR = 1.0 / WHEEL_CIRCUMFERENCE_M;
        public static final double TALONFX_RPS_TO_MPS_FACTOR = WHEEL_CIRCUMFERENCE_M;
        public static final double MPS_TO_TALONFX_RPS_FACTOR = 1.0 / WHEEL_CIRCUMFERENCE_M;
        public static final double ANGLE_TO_ROTATION_FACTOR = 1.0 / 360.0;
        public static final double FALCON_GEAR_RATIO = 150.0 / 7.0;

        // Current Limiting motor protection - same for both module types
        public static final double  DRIVE_SUPPLY_CURRENT_LIMIT          = 60.0;
        public static final double  DRIVE_SUPPLY_CURRENT_THRESHOLD      = 0.0;
        public static final double  DRIVE_SUPPLY_CURRENT_TIME_THRESHOLD = 0.1;
        public static final boolean DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT   = true;
        public static final double  DRIVE_STATOR_CURRENT_LIMIT          = 100.0;
        public static final boolean DRIVE_ENABLE_STATOR_CURRENT_LIMIT   = true;

        public static final double STEER_SUPPLY_CURRENT_LIMIT = 40.0;
        public static final double STEER_SUPPLY_CURRENT_THRESHOLD = 0.0;
        public static final double STEER_SUPPLY_CURRENT_TIME_THRESHOLD = 0.1;
        public static final boolean STEER_ENABLE_SUPPLY_CURRENT_LIMIT = true;
        public static final double STEER_STATOR_CURRENT_LIMIT = 60.0;
        public static final boolean STEER_ENABLE_STATOR_CURRENT_LIMIT = true;
        
        // Voltage compensation
        public static final double STEER_MOTOR_VOLTAGE_COMPENSATION = 12.0;

        // These values are used by the drive motor to ramp in open loop.
        // Team 364 found a small open loop ramp (0.25) helps with tread wear, 
        // avoiding tipping, etc. In closed loop control, it would probably be 
        // better to employ profiled PID controllers.
        public static final double OPEN_LOOP_RAMP_PERIOD = 1.00;
        public static final double CLOSED_LOOP_RAMP_PERIOD = 0.0;

        // Drive Motor PID Values
        public static final double DRIVE_KP = 0.1;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        // Drive Motor Characterization Values
        // Divide SYSID values by 12 to convert from volts to percent output for CTRE
        public static final double DRIVE_KS = (0.32 / 12);
        public static final double DRIVE_KV = (1.51 / 12);
        public static final double DRIVE_KA = (0.27 / 12);
        public static final double DRIVE_KG = 0.27;
        
        // Steer motor characterization values.
        public static final double STEER_KS = (0.32 /12);
        public static final double STEER_KV = (1.51 /12);
        public static final double STEER_KA = (0.27 /12);
        public static final double STEER_KG = 0.27;

        // Swerve Profiling Values for Robot
        // Best if getten by characterizing the robot, but these values worked
        // tolerably well in 2023 as is. Used during Auto mode moves, which are
        // tracked by Odometry.
        public static final double MAX_ROBOT_SPEED_M_PER_SEC = 4.5; // 4.96 theoretically 
        public static final double MAX_ROBOT_ANG_VEL_RAD_PER_SEC = 11.0; // 11.96 theoretically 

        // Swerve output fixed limit values for teleop control (reduce if
        // speeds are too fast for the experience level of the drive team).
        // (In Auto, tuning should set speeds to reasonable values, no need
        // to reduce them - in fact, just the opposite, want fastest possible
        // movements in Auto mode, consistent with safety).
        public static final double OUTPUT_DRIVE_LIMIT_FACTOR = .75;
        public static final double OUTPUT_ROTATE_LIMIT_FACTOR = .75;

        // When monitored while set at -1 to 1, seemed like the Steering PID output 
        // did not generate percent outputs greater than about .4
        public static final double MIN_STEER_CLOSED_LOOP_OUTPUT = -0.6;
        public static final double MAX_STEER_CLOSED_LOOP_OUTPUT = 0.6;

        /* Default Motor Neutral Modes */
        public static final NeutralModeValue STEER_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast; // SparkBaseConfig.IdleMode.kCoast;
        public static final NeutralModeValue DRIVE_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;

        // Swerve rotate speed and extent during a parade WAVE function
        public static final double WAVE_SWERVE_ROTATE_SPEED = 0.1;      // Percent simulated joystick input
        public static final double WAVE_ROTATION_EXTENT = 70;           // Degree units per side, relative to 0 heading
        public static final long   WAVE_ROTATION_PAUSE_IN_MS = 2500;    // Time in MS to stop rotating at limit
                                                                        // and wave faster, before reverting to
                                                                        // normal wave and rotating to other side
                                                                        // See WaveCmd.java
                                                                        
        // Finally, declare constants to define and allow addressing the (typically 4)
        // individual module components, including the Shuffleboard coordinates
        // embedded in the associated ShuffleboardLayout objects created for each
        // module under the shared "SwerveDrive" Shuffleboard Tab
        public static final int FIRST_SWERVE_MOD_LIST_COL = 5;
        public static final int FIRST_SWERVE_MOD_LIST_ROW = 0;
        public static final int SWERVE_MOD_LIST_HGT = 4;
        public static final ShuffleboardTab sbt = Shuffleboard.getTab("SwerveDrive");

        // Front Left Module - Module 0
        public static final class FL_Mod0 {
            public static final int driveMotorID = 1;
            public static final int steerMotorID = 2;
            public static final int canCoderID   = 1;
            public static final Rotation2d angleOffset = ABS_OFFSET_FL;
            public static final ShuffleboardLayout sBE_Layout0 = 
                                    sbt.getLayout("FL_Mod0", BuiltInLayouts.kList)
                                       .withPosition(FIRST_SWERVE_MOD_LIST_COL + 0,
                                                     FIRST_SWERVE_MOD_LIST_ROW)
                                       .withSize(1, SWERVE_MOD_LIST_HGT)
                                       .withProperties(Map.of("Label position", "LEFT"));
            public static final SwerveModuleConstants MODULE_CONSTANTS = 
                                        new SwerveModuleConstants(driveMotorID, 
                                                                  steerMotorID, 
                                                                  canCoderID, 
                                                                  angleOffset,
                                                                  sBE_Layout0);
        }

        // Front Right Module - Module 1
        public static final class FR_Mod1 {
            public static final int driveMotorID = 3;
            public static final int steerMotorID = 4;
            public static final int canCoderID   = 2;
            public static final Rotation2d angleOffset = ABS_OFFSET_FR;                             
            public static final ShuffleboardLayout sBE_Layout1 = 
                                     sbt.getLayout("FR_Mod1", BuiltInLayouts.kList)
                                        .withPosition(FIRST_SWERVE_MOD_LIST_COL + 1,
                                                    FIRST_SWERVE_MOD_LIST_ROW)
                                        .withSize(1, SWERVE_MOD_LIST_HGT)
                                        .withProperties(Map.of("Label position", "LEFT"));
            public static final SwerveModuleConstants MODULE_CONSTANTS = 
                                        new SwerveModuleConstants(driveMotorID, 
                                                                  steerMotorID, 
                                                                  canCoderID, 
                                                                  angleOffset,
                                                                  sBE_Layout1);
        }
        
        /* Back Left Module - Module 2 */
        public static final class BL_Mod2 {
            public static final int driveMotorID = 5;
            public static final int steerMotorID = 6;
            public static final int canCoderID   = 3;
            public static final Rotation2d angleOffset = ABS_OFFSET_BL;
            public static final ShuffleboardLayout sBE_Layout2 = 
                                    sbt.getLayout("BL_Mod2", BuiltInLayouts.kList)
                                       .withPosition(FIRST_SWERVE_MOD_LIST_COL + 2,
                                                     FIRST_SWERVE_MOD_LIST_ROW)
                                       .withSize(1, SWERVE_MOD_LIST_HGT)
                                       .withProperties(Map.of("Label position", "LEFT"));
            public static final SwerveModuleConstants MODULE_CONSTANTS = 
                                        new SwerveModuleConstants(driveMotorID, 
                                                                  steerMotorID, 
                                                                  canCoderID, 
                                                                  angleOffset,
                                                                  sBE_Layout2);
        }

        /* Back Right Module - Module 3 */
        public static final class BR_Mod3 {
            public static final int driveMotorID = 7;
            public static final int steerMotorID = 8;
            public static final int canCoderID   = 4;
            public static final Rotation2d angleOffset = ABS_OFFSET_BR;
            public static final ShuffleboardLayout sBE_Layout3 = 
                                    sbt.getLayout("BR_Mod3", BuiltInLayouts.kList)
                                       .withPosition(FIRST_SWERVE_MOD_LIST_COL + 3,
                                                     FIRST_SWERVE_MOD_LIST_ROW)
                                       .withSize(1, SWERVE_MOD_LIST_HGT)
                                       .withProperties(Map.of("Label position", "LEFT"));
            public static final SwerveModuleConstants MODULE_CONSTANTS = 
                                        new SwerveModuleConstants(driveMotorID, 
                                                                  steerMotorID, 
                                                                  canCoderID, 
                                                                  angleOffset,
                                                                  sBE_Layout3);
        }
    }

    /***********************************************************
     * Autonomous Constants
     ***********************************************************/
    public static final class AutoC {       // AutoC = short for AutoConstants
        public static final double AUTO_MAX_SPEED_M_PER_SEC = 4.5;
        public static final double AUTO_MAX_ACCEL_M_PER_SEC2 = 3.5;      // was 3.0
        public static final double AUTO_MAX_ANG_VEL_RAD_PER_SEC = 6.5*Math.PI;
        public static final double AUTO_MAX_ANG_ACCEL_RAD_PER_SEC2 = 3.5*Math.PI;

        public static final double KP_X_CONTROLLER = 20.0;
        public static final double KI_X_CONTROLLER = 0.5;
        public static final double KP_Y_CONTROLLER = 20.0;
        public static final double KI_Y_CONTROLLER = 0.5;
        public static final double KP_THETA_CONTROLLER = 12.0;
        public static final double KI_THETA_CONTROLLER = 0.5;
    
        /* Constraint for the motion profiled robot angle controller */
        public static final TrapezoidProfile.Constraints K_THETA_CONTROLLER_CONSTRAINTS =
                            new TrapezoidProfile.Constraints(AUTO_MAX_ANG_VEL_RAD_PER_SEC, 
                                                             AUTO_MAX_ANG_ACCEL_RAD_PER_SEC2);

        public static final double AUTO_SPEED_FACTOR_GENERIC = 1.0;
        public static final double AUTO_ACCEL_FACTOR_GENERIC = 1.0;
    }

    /***********************************************************
     * Coral Arm Constants 
     ***********************************************************/
    public static final class CAC {
        //TODO: add value for servo PWM channel
        public static final int CORAL_MOTOR_CAN_ID = 11;
        public static final int CORAL_CANCODER_ID = 11;
        public static final int CORAL_ARM_SERVO_CAN_ID = 12;

        public static final double CORAL_ARM_CANCODER_RANGE = .275;
        public static final double CORAL_ARM_CLOSED_LOOP_RAMP_PERIOD = 0.5;
        public static final double CORAL_ARM_CANCODER_MAGNET_OFFSET = -0.2;

        public static final SensorDirectionValue CORAL_ARM_CANCODER_DIR = SensorDirectionValue.CounterClockwise_Positive;
        public static final double CORAL_ARM_CANCODER_TO_AXLE_RATIO = 20.0/18.0;
        public static final double CORAL_ARM_ROTOR_TO_CANCODER_RATIO = 30;

        public static final NeutralModeValue CORAL_ARM_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final InvertedValue CORAL_ARM_MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        public static final double CORAL_ARM_OUTPUT_LIMIT_FACTOR = 1.0;
        public static final double CORAL_ARM_CONT_CURRENT_LIMIT = 16.0;
        public static final double CORAL_ARM_PEAK_CURRENT_LIMIT = 36.0;
        public static final double CORAL_ARM_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean CORAL_ARM_ENABLE_CURRENT_LIMIT = true;

        public static final double CORAL_ARM_KP = 40.0;
        public static final double CORAL_ARM_KI = 0.0;
        public static final double CORAL_ARM_KD = 0.0;
        public static final double CORAL_ARM_KS = 0.225;
        public static final double CORAL_ARM_KV = 0.5;
        public static final double CORAL_ARM_KA = 4.0;
        public static final double CORAL_ARM_KG = 0.49;

        public static final double CORAL_ARM_MOTION_MAGIC_VEL = 5.0;
        public static final double CORAL_ARM_MOTION_MAGIC_ACCEL = 8.0;
        public static final double CORAL_ARM_MOTION_MAGIC_JERK = 25.0;
        public static final double CORAL_ARM_MOTION_MAGIC_kA = 0.0;
        public static final double CORAL_ARM_MOTION_MAGIC_kV = 0.0;

        public static final double CORAL_ARM_SOURCE_POSITION = -0.31; 
        public static final double CORAL_ARM_SCORE_POSITION = -0.13;  
        public static final double CORAL_ARM_SCORE_L1_POSITION = -0.13;
        public static final double CORAL_ARM_CENTER_POSITION = -0.25;
        public static final double PIN_SERVO_OPEN_POSITION = 0.11; // 0 for other servo orientation 
        public static final double PIN_SERVO_CLOSED_POSITION = 0; // 0.25 for other servo orientation

        public static final double CORAL_ARM_MECHANISM_RATIO = 1;

        public static final double PIN_SERVO_MAX = 2.0;
        public static final double PIN_SERVO_CENTER = 1.5;
        public static final double PIN_SERVO_DEADBAND = 0.002;
        public static final double PIN_SERVO_MIN = 0.8;

            //placeholder value
        public static final int CORAL_ARM_SERVO_PWM_CHANNEL = 0;


    }

    /***********************************************************
     * Algae Arm Constants 
     ***********************************************************/
    public static final class AAC {
        public static final int ALGAE_MOTOR_CAN_ID = 13;
        public static final int ALGAE_CANCODER_ID = 13;
        public static final int ALGAE_WHEEL_MOTOR_CAN_ID = 14;
        public static final int ALGAE_WHEEL_CANCODER_ID = 14;

        public static final int ALGAE_ARM_CANCODER_RANGE = 1;
        public static final double ALGAE_ARM_CLOSED_LOOP_RAMP_PERIOD = 0.5;
        public static final double ALGAE_ARM_CANCODER_MAGNET_OFFSET = 0.21096;

        public static final SensorDirectionValue ALGAE_ARM_CANCODER_DIR = SensorDirectionValue.CounterClockwise_Positive;
        public static final double ALGAE_ARM_CANCODER_TO_AXLE_RATIO = 20.0/18.0;
        public static final double ALGAE_ARM_ROTOR_TO_CANCODER_RATIO = 30;

        public static final NeutralModeValue ALGAE_ARM_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final InvertedValue ALGAE_ARM_MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        public static final double ALGAE_ARM_OUTPUT_LIMIT_FACTOR = 1.0;
        public static final double ALGAE_ARM_CONT_CURRENT_LIMIT = 16.0;
        public static final double ALGAE_ARM_PEAK_CURRENT_LIMIT = 36.0;
        public static final double ALGAE_ARM_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ALGAE_ARM_ENABLE_CURRENT_LIMIT = true;

        public static final double ALGAE_ARM_KP = 0.5;
        public static final double ALGAE_ARM_KI = 0.0;
        public static final double ALGAE_ARM_KD = 0.0;
        public static final double ALGAE_ARM_KS = 0.25;
        public static final double ALGAE_ARM_KV = 0;
        public static final double ALGAE_ARM_KA = 0;
        public static final double ALGAE_ARM_KG = 0;

        public static final double ALGAE_ARM_MOTION_MAGIC_VEL = 5.0;
        public static final double ALGAE_ARM_MOTION_MAGIC_ACCEL = 8.0;
        public static final double ALGAE_ARM_MOTION_MAGIC_JERK = 25.0;
        public static final double ALGAE_ARM_MOTION_MAGIC_kA = 0.0;
        public static final double ALGAE_ARM_MOTION_MAGIC_kV = 0.0;

        public static final double ALGAE_WHEEL_CLOSED_LOOP_RAMP_PERIOD = 0.5;
        public static final double ALGAE_WHEEL_CANCODER_MAGNET_OFFSET = 0.21096;

        public static final SensorDirectionValue ALGAE_WHEEL_CANCODER_DIR = SensorDirectionValue.CounterClockwise_Positive;
        public static final double ALGAE_WHEEL_CANCODER_TO_AXLE_RATIO = 20.0/18.0;
        public static final double ALGAE_WHEEL_ROTOR_TO_CANCODER_RATIO = 30;

        public static final NeutralModeValue ALGAE_WHEEL_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final InvertedValue ALGAE_WHEEL_MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        public static final double ALGAE_WHEEL_OUTPUT_LIMIT_FACTOR = 1.0;
        public static final double ALGAE_WHEEL_CONT_CURRENT_LIMIT = 16.0;
        public static final double ALGAE_WHEEL_PEAK_CURRENT_LIMIT = 36.0;
        public static final double ALGAE_WHEEL_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ALGAE_WHEEL_ENABLE_CURRENT_LIMIT = true;
            public static final SupplyCurrentLimitConfiguration ALGAE_WHEEL_CURRENT_LIMIT = 
            new SupplyCurrentLimitConfiguration(
                ALGAE_WHEEL_ENABLE_CURRENT_LIMIT,                      // launchEnableCurrentLimit, 
                ALGAE_WHEEL_CONT_CURRENT_LIMIT,                  // launchContinuousCurrentLimit,
                ALGAE_WHEEL_PEAK_CURRENT_LIMIT,       // launchPeakCurrentLimit, 
                ALGAE_WHEEL_PEAK_CURRENT_DURATION);        // launchPeakCurrentDuration);

        public static final double ALGAE_WHEEL_KP = 10.0;
        public static final double ALGAE_WHEEL_KI = 0.0;
        public static final double ALGAE_WHEEL_KD = 0.0;
        public static final double ALGAE_WHEEL_KS = 0.2;
        public static final double ALGAE_WHEEL_KV = 0.0;
        public static final double ALGAE_WHEEL_KA = 0.0;
        public static final double ALGAE_WHEEL_KG = 0.2;
        public static final double ALGAE_WHEEL_KF = 0.0;

        public static final double ALGAE_WHEEL_MOTION_MAGIC_VEL = 5.0;
        public static final double ALGAE_WHEEL_MOTION_MAGIC_ACCEL = 8.0;
        public static final double ALGAE_WHEEL_MOTION_MAGIC_JERK = 25.0;
        public static final double ALGAE_WHEEL_MOTION_MAGIC_kA = 0.0;
        public static final double ALGAE_WHEEL_MOTION_MAGIC_kV = 0.0;

        public static final long ALGAE_ARM_L2_POSITION = -20;
        public static final long ALGAE_ARM_L3_POSITION = 0;
        public static final long ALGAE_ARM_PICKUP_POSITION = -62;
        public static final long ALGAE_ARM_UP_POSITION = 0;

    }

    /*************************************************
     * Elevator Constants 
    **************************************************/
    public static final class EC {
        public static final int ELEVATOR_MOTOR_CAN_ID = 10;
        public static final int ELEVATOR_CANCODER_ID = 10;

        public static final int ELEVATOR_CANCODER_RANGE = 1;
        public static final double ELEVATOR_CLOSED_LOOP_RAMP_PERIOD = 0.5;
        public static final double ELEVATOR_CANCODER_MAGNET_OFFSET = 0.89797;

        public static final SensorDirectionValue ELEVATOR_CANCODER_DIR = SensorDirectionValue.CounterClockwise_Positive;
        public static final double ELEVATOR_CANCODER_TO_AXLE_RATIO = 20.0/18.0;
        public static final double ELEVATOR_ROTOR_TO_CANCODER_RATIO = 30;

        public static final double INCHES_TO_ROTATION_FACTOR = 3.375;

        public static final NeutralModeValue ELEVATOR_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final InvertedValue ELEVATOR_MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        public static final double ELEVATOR_OUTPUT_LIMIT_FACTOR = 1.0;
        public static final double ELEVATOR_CONT_CURRENT_LIMIT = 16.0;
        public static final double ELEVATOR_PEAK_CURRENT_LIMIT = 36.0;
        public static final double ELEVATOR_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ELEVATOR_ENABLE_CURRENT_LIMIT = true;

        public static final double ELEVATOR_KP = 8.0;
        public static final double ELEVATOR_KI = 0.0;
        public static final double ELEVATOR_KD = 0.25;
        public static final double ELEVATOR_KS = 0.225;
        public static final double ELEVATOR_KV = 0.0;
        public static final double ELEVATOR_KA = 0.0;
        public static final double ELEVATOR_KG = 0.2;

        public static final double ELEVATOR_MOTION_MAGIC_VEL = 5.0;
        public static final double ELEVATOR_MOTION_MAGIC_ACCEL = 8.0;
        public static final double ELEVATOR_MOTION_MAGIC_JERK = 25.0;
        public static final double ELEVATOR_MOTION_MAGIC_kA = 0.0;
        public static final double ELEVATOR_MOTION_MAGIC_kV = 0.0;

        public static final double ELEVATOR_SOURCE_POSITION = -1.6;
        public static final double ELEVATOR_L2_SOURCE_POSITION = -1.6;
        public static final double ELEVATOR_L1_CORAL_POSITION = 0;
        public static final double ELEVATOR_L2_CORAL_POSITION = -1.6;
        public static final double ELEVATOR_L3_CORAL_POSITION = -5.75;
        public static final double ELEVATOR_L2_ALGAE_POSITION = 0;
        public static final double ELEVATOR_L3_ALGAE_POSITION = 0;
        public static final double ELEVATOR_ALGAE_PICKUP_POSITION = 0;
    }

    /***********************************************************
     * Climb Subsystem Constants
     ***********************************************************/
    public static final class CSC {
        //TODO: set value for servo PWM channels
        public static final int WINCH_MOTOR_CAN_ID = 15;
        public static final int LINEAR_SERVO_PWM_CHANNEL = 1;
        public static final int HOOK_SERVO_PWM_CHANNEL = 2;
        public static final int SPRING_SERVO_PWM_CHANNEL = 3;
        
        public static final double WINCH_CLOSED_LOOP_RAMP_PERIOD = 0.5;
        public static final double WINCH_CANCODER_TO_AXLE_RATIO = 20/18;
        public static final double WINCH_ROTOR_TO_CANCODER_RATIO = 30.0;
        public static final NeutralModeValue WINCH_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final InvertedValue WINCH_MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        public static final double WINCH_OUTPUT_LIMIT_FACTOR = 1.0;
        public static final double WINCH_CONT_CURRENT_LIMIT = 16.0;
        public static final double WINCH_PEAK_CURRENT_LIMIT = 36.0;
        public static final double WINCH_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean WINCH_ENABLE_CURRENT_LIMIT = true;

        public static final double WINCH_KP = 40.0;
        public static final double WINCH_KI = 0.5; 
        public static final double WINCH_KD = 0.1;
        public static final double WINCH_KS = 0.3;
        public static final double WINCH_KV = 4.0;
        public static final double WINCH_KA = 0.1;
        public static final double WINCH_KG = 0.5;

        public static final double WINCH_MOTION_MAGIC_VEL = 5.0;
        public static final double WINCH_MOTION_MAGIC_ACCEL = 8.0;
        public static final double WINCH_MOTION_MAGIC_JERK = 25.0;
        public static final double WINCH_MOTION_MAGIC_kA = 0.0;
        public static final double WINCH_MOTION_MAGIC_kV = 0.0;

        public static final double HINGE_SERVO_OPEN_POSITION = 0.0;
        public static final double HINGE_SERVO_CLOSED_POSITION = 0.0;
        public static final double HOOK_SERVO_CLOSED_POSITION = 0.0;
        public static final double HOOK_SERVO_OPEN_POSITION = 0.0;
        public static final double SPRING_SERVO_DISENGAGED_POSITION = 0.0;
        public static final double SPRING_SERVO_ENGAGED_POSITION = 0.0;

        public static final double CLIMB_WINCH_TIMEOUT = 0.0;
    }
}
