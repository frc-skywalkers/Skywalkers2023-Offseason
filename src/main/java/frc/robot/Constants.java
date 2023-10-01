package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.lightstrip.LedState;
import frc.robot.lightstrip.TempLedState;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 13;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(28.00); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(28.00); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class OldAutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 3;
        public static final double kPXController = 5;
        public static final double kPYController = 5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);

        // key: 
        // 0: home position
        // 1: intake from substation
        // 2: cone 2nd
        // 3: cone 3rd
        // 4: cube 2nd
        // 5: cube 3rd
        // 6: ground intake

        public static final double[] armPreset = new double[]{1.33, 0, 0.8, 0.47, 0, 0, -0.19};
        public static final double[] elevatorPreset = new double[]{0, 1.13, 0.72, 1.38, 0.85, 1.26, 0.10};
    }

    public static final class Presets {
        public static final Preset STOW_PRESET = new Preset(1.6, 0); // done
        public static final Preset GROUND_INTAKE_PRESET = new Preset(-0.19, 0.10); // don't use
        public static final Preset GROUND_INTAKE_CONE_PRESET = new Preset(0.22, 0);     
        public static final Preset GROUND_INTAKE_CUBE_PRESET = new Preset(0.1, 0.01); // done
        public static final Preset SUBSTATION_INTAKE_CONE_PRESET = new Preset(0.06, 1.25);
        public static final Preset SUBSTATION_INTAKE_PRESET = new Preset(0.0, 1.13);
        public static final Preset SUBSTATION_INTAKE_CUBE_PRESET = new Preset(0.45, 1.05);
        public static final Preset CONE_2ND_STAGE_PRESET = new Preset(0.54, 0.63);
        public static final Preset CONE_3RD_STAGE_PRESET = new Preset(0.48, 1.16);
        public static final Preset CUBE_2ND_STAGE_PRESET = new Preset(0.81, 0.60);
        public static final Preset CUBE_3RD_STAGE_PRESET = new Preset(0.57, 1.22);   
        public static final Preset SINGLE_SUBSTATION_CUBE = new Preset(1.85, 0);
        public static final Preset SINGLE_SUBSTATION_CONE = new Preset(0.72, 0.68);
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameter = Units.inchesToMeters(4); // done
        public static final double kTicksPerRotation = 2048 * 150 / 7; // done
        public static final double kDriveTicksPerRotation = 2048 * 6.75;

        public static final double kPTurning = 0.65; // CHANGE!!!
        public static final double kDTurning = 0.025; // necessary??
        public static final double kITurning = 0.0;

        public static final double kVDrive = 2.16;
        public static final double kSDrive = 0.634;

   }   

    public static final class OIConstants {
        public static final double kDeadband = 0.15;

        public static final int kDriverControllerPort = 0;
        public static final int kDriverControllerPort2 = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(24);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(24);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.00;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    
        public static final int kFrontLeftDriveMotorPort = 10;
        public static final int kBackLeftDriveMotorPort = 7;
        public static final int kFrontRightDriveMotorPort = 1;
        public static final int kBackRightDriveMotorPort = 4;

        public static final int kFrontLeftTurningMotorPort = 11;
        public static final int kBackLeftTurningMotorPort = 8;
        public static final int kFrontRightTurningMotorPort = 2;
        public static final int kBackRightTurningMotorPort = 5;

        // MIGHT NEED TO CHANGE
        public static final boolean kFrontLeftTurningEncoderReversed = true; //true
        public static final boolean kBackLeftTurningEncoderReversed = true; //true
        public static final boolean kFrontRightTurningEncoderReversed = true; //true
        public static final boolean kBackRightTurningEncoderReversed = true; //true

        // MIGHT NEED TO CHANGE
        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 12;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 9;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
        public static final int kBackRightDriveAbsoluteEncoderPort = 6;

        // MIGHT NEED TO CHANGE
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        // public static final double kFLAbsEcoderOffsetDeg = 81.5;
        // public static final double kBLAbsEcoderOffsetDeg = 160.2;
        // public static final double kFRAbsEncoderOffsetDeg = 22.7;
        // public static final double kBRAbsEncoderOffsetDeg = 168.8;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 2.96;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 4.377;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 5.140;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 4.493;

        public static final int kIMUPort = 13;


        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4; // ????


        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3; // ????
    
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 5.000 / 5.0000; // ?????
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond/3; //????
    }

    public static final class IntakeConstants {
        public static final int kIntakePort = 22; // ??
        public static final double kMaxIntakeSpeed = 0.8; // ??
        public static final double kMaxOuttakeSpeed = -0.3; // ??
        public static final double kHoldSpeed = 0.08;

        public static double kSpeedUpFailTime = 0.75; //seconds it tries to speed up
        public static double kOutFailTime = 0.75; //seconds it tries to outtake

        public static double conePieceHeldThreshold = 500;
        public static double cubePieceHeldThreshold = 750;
        public static boolean differentialIntake = true;
        public static int tofPort = 15;
    
        public static double threshold(double speed) {
            speed = Math.abs(speed);
            return (speed * 20850) - 1365;
        }
    }

    public static final class ElevatorConstants {
        public static final int kLeftElevatorPort = 31;
        public static final int kRightElevatorPort = 30; 

        public static final double kMaxElevatorSpeed = 0.4;
        public static final boolean kLeftElevatorInverted = false;
        public static final boolean kRightElevatorInverted = true;
        public static final int kelevatorTicksPerRotation = 2048;
        public static final double kElevatorGearRatio = 10.00 * 22.0 / 12.0;
        public static final double kSpoolDiameter = Units.inchesToMeters(1.751);
        public static final double kDistancePerRevolution = Math.PI * kSpoolDiameter;

        public static final double kPositionConversionFactor = kDistancePerRevolution/(kElevatorGearRatio * kelevatorTicksPerRotation) * 2.53715;
        public static final double kVelocityConversionFactor = kPositionConversionFactor * 10.0;

        public static final double kBottomLimit = 0.0;
        public static final double kTopLimit = 1.45;

        public static final double kMaxVel = 1.5;
        public static final double kMaxAcc = 1.0;

        public static final double kVUp = 6.17;
        public static final double kSUp = 0.999;

        public static final double kVDown = 6.41;
        public static final double kSDown = -0.13;
    
    

        public static final double kP = 30.00;
        public static final double kI = 0.00;
        public static final double kD = 0.00;

        public static final double kCurrentThreshold = 10.00;
        public static final double kHomingSpeed = -0.07;

        public static final double kMountAngleRadians = 0.9599;
    }


    public static final class NewArmConstants {
        public static final int kArmPort = 23; // ???
        public static final int kArmAbsoluteEncoderPort = 16;
        public static final double kMaxArmSpeed = 0.25; // ??
        public static final boolean kArmInverted = true; // ??
        public static final boolean kArmAbsEncoderInverted = false;
        // public static final boolean kEncoderInverted = true;
    
        public static final int kArmTicksPerRotation = 2048; // ??
        public static final double kArmGearRatio = 45.0 * 44.0 / 18.0;

        public static final double kAbsEncoderOffset = -73.5;


        public static final double kPositionConversionFactor = 2 * Math.PI / (kArmTicksPerRotation * kArmGearRatio);
        public static final double kVelocityConversionFactor = kPositionConversionFactor * 10.0;

        public static final double kBottomLimit = -0.3; 
        public static final double kTopLimit = 1.85; 

        // feedback

        public static final double kPArm = 10.00;
        public static final double kIArm = 0.0;
        public static final double kDArm = 0.00;

        // feedforward

        // calculate necessary gains using recalc: https://www.reca.lc/linear (overkill?)

        // public static final double kSArm = 0.00; // to overcome static friction
        // public static final double kVArm = 2.69; //1.47 for Neo550
        // public static final double kGArm = 0.07; //0.32 for Neo550
        // public static final double kAArm = 0.00;

        public static final double kVUp = 1.95;
        public static final double kSUp = 0.613;

        public static final double kVDown = 1.92;
        public static final double kSDown = -0.424;

        public static final double kCurrentThreshold = 15.00;


    }



    public static final class LimelightConstants{
        public static double kPx = 5; //meters
        public static double kPy = 5; //meters
        public static double kPr = 0.2; //degrees
        public static double kDx = 0;
        public static double kDy = 0;
        public static double kDr = 0;
        public static double kIx = 0;
        public static double kIy = 0;
        public static double kIr = 0;
  
        public static double xclamp = 0.8; //maximum clamped speed is 0.6 + 0.5 (min)
        public static double yclamp = 0.8;
        public static double rclamp = 2;
  
        //public static double tagheight = 0.49; //19.3 inches to meters
  
        public static double xtolerance = 0.02; 
        public static double ytolerance = 0.02; 
        public static double rtolerance = 2; //degrees
  
        public static double mountingangle = 0; //for adjustable camera
        public static double cameraheight = Units.inchesToMeters(17.5); //14-ish inches, to meters, REDO
        public static double RTheight = Units.inchesToMeters(22.55); //game manual 24.125
        public static double objectHeight = Units.inchesToMeters(0);

        public static double limelightOffsetCenter = Units.inchesToMeters(10.5);

        public static final class SecondStageConeConstants{
            public static double targetXMeters = 0.92;
            public static double targetYMeters = 0;
            public static double targetRDeg = 0;
        }
    }

    public static final class DashboardConstants {
        public static boolean SwerveDebugging = true;
        public static boolean SwerveDriver = true;
        public static boolean ArmDebugging = true;
        public static boolean ArmDriver = true;
        public static boolean ElevatorDebugging = true;
        public static boolean ElevatorDriver = true;
        public static boolean IntakeDebugging = true;
        public static boolean IntakeDriver = true;
        public static boolean AutoDebugging = true;
        public static boolean AutoDriver = true;
        public static boolean TeleDebugging = true;
        public static boolean TeleDriver = true;
        public static boolean LimelightDebugging = true;
        public static boolean LimelightDriver = true;
    }

    public static final class lightstripConstants {
        public static int redPort = 1;
        public static int greenPort = 2;
        public static int bluePort = 0;

        public static LedState defaultState = new LedState(255, 0, 0, "Solid");
        public static TempLedState successSignal = new TempLedState(0, 255, 0, "Solid", 2);
        public static LedState coneIntake = new LedState(255, 200, 0, "Solid");
        public static LedState cubeIntake = new LedState(195, 0, 255, "Solid");
    }

    public static final class TofConstants {
        public static final int secondaryPort = 16;
        public static final int primaryPort = 15;

        public static final boolean secondaryMode = true; //true for cone, false for cube

        public static final class width { //precise for alignment
            public static final double cube = 10;
            public static final double cone = 5;
        }

        public static final class full { //inprecise to just tell if cube or cone is inside
            public static final double cube = 30;
            public static final double cone = 5;
        }

        public static final class halfway {
            public static final double cube = 15;
            public static final double cone = 15;
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
