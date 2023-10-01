package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.NewArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.lightstripConstants;
import frc.robot.commands.ArmCharacterization;
import frc.robot.commands.IntakePiece;
import frc.robot.commands.Macros;
import frc.robot.commands.OuttakePiece;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Lightstrip;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ProfiledPIDElevator;
import frc.robot.subsystems.Limelight;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final Swerve s_Swerve = new Swerve();
    public final ProfiledPIDElevator elevator = new ProfiledPIDElevator();
    public final ArmSubsystem arm = new ArmSubsystem();
    private final Lightstrip lightstrip = new Lightstrip();
    private final TofSubsystem tof = new TofSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem(lightstrip, tof);
    private final Limelight limelight = new Limelight();
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    SendableChooser<Command> m_Chooser = new SendableChooser<>();

    private final CommandXboxController driverJoystick = new CommandXboxController(OIConstants.kDriverControllerPort);
    private final CommandXboxController operatorJoystick = new CommandXboxController(OIConstants.kDriverControllerPort2);
    
    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    // private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    /* Subsystems */
    private final Macros macros = new Macros(s_Swerve, elevator, arm, intake, limelight);
    //private final AutoRoutines autoRoutines = new AutoRoutines(swerve, elevator, arm, intake, limelight, lightstrip);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        startDashboard();
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        elevator.setDefaultCommand(Commands.run(() -> {
            double speed = -operatorJoystick.getLeftY() * ElevatorConstants.kMaxElevatorSpeed;
            elevator.setSpeed(speed);
          }, elevator).unless(elevator::isEnabled));
        
        arm.setDefaultCommand(Commands.run(() -> {
            double speed = -operatorJoystick.getRightY() * NewArmConstants.kMaxArmSpeed;
            speed = Math.abs(speed) > OIConstants.kDeadband ? speed : 0.0;
            arm.setSpeed(speed);
            }, arm).unless(arm::isEnabled));
        
        /*
        m_Chooser.setDefaultOption("3rd Stage Cube Balance", autoRoutines.chargingStation());
        m_Chooser.addOption("3rd Stage Cone Balance", autoRoutines.Cone3rdBalance());
        m_Chooser.addOption("2nd Stage Cone Balance", autoRoutines.coneChargingStation());
        m_Chooser.addOption("2nd Stage Cube Balance", autoRoutines.Cube2ndBalance());
        
        m_Chooser.addOption("3rd Stage Cube", autoRoutines.cube3rdAuto());
        m_Chooser.addOption("3rd Stage Cone", autoRoutines.cone3rdAuto());
        m_Chooser.addOption("2nd Stage Cone", autoRoutines.cone2ndAuto());
        m_Chooser.addOption("2nd Stage Cube", autoRoutines.cube2ndAuto());
        
        m_Chooser.addOption("BLUE 2 Piece Auto", autoRoutines.blueConeCubeAuto());
        m_Chooser.addOption("RED 2 Piece Auto", autoRoutines.redConeCubeAuto());
        */
         
        SmartDashboard.putData(m_Chooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    private void startDashboard() {
        Dashboard.Swerve.Debugging.set(DashboardConstants.SwerveDebugging);
        Dashboard.Swerve.Driver.set(DashboardConstants.SwerveDriver);
        Dashboard.Elevator.Debugging.set(DashboardConstants.ElevatorDebugging);
        Dashboard.Elevator.Driver.set(DashboardConstants.ElevatorDriver);
        Dashboard.Intake.Debugging.set(DashboardConstants.IntakeDebugging);
        Dashboard.Intake.Driver.set(DashboardConstants.IntakeDriver);
        Dashboard.Auto.Debugging.set(DashboardConstants.AutoDebugging);
        Dashboard.Auto.Driver.set(DashboardConstants.AutoDriver);
        Dashboard.Tele.Debugging.set(DashboardConstants.TeleDebugging);
        Dashboard.Tele.Driver.set(DashboardConstants.TeleDriver);
        Dashboard.Limelight.Debugging.set(DashboardConstants.LimelightDebugging);
        Dashboard.Limelight.Driver.set(DashboardConstants.LimelightDriver);
        Dashboard.Arm.Debugging.set(DashboardConstants.ArmDebugging);
        Dashboard.Arm.Driver.set(DashboardConstants.ArmDriver);
      }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        driverJoystick.y().onTrue(Commands.runOnce(() -> s_Swerve.zeroGyro(), s_Swerve));

        //driverJoystick.b().onTrue(Commands.runOnce(() -> swerve.toggleField(), swerve));

        //driverJoystick.x().onTrue(Commands.runOnce(() -> swerve.setHeading(180.000)));
        //driverJoystick.leftBumper().onTrue(Commands.runOnce(() -> swerve.stopModules(), swerve));
        // driverJoystick.x().onTrue(macros.alignCone2ndStage());

        // driverJoystick.a().onTrue(new Balance(swerve));
        //driverJoystick.rightBumper().onTrue(Commands.runOnce(swerve::stopModules, swerve));

        // // Right Trigger --> manual override
        operatorJoystick.rightTrigger().onTrue(
            Commands.runOnce(() -> {
            arm.stop();
            arm.disable();
            elevator.stop();
            elevator.disable();
        }, arm, elevator));

        // Start --> Home
        operatorJoystick.start().onTrue(macros.home());

    
        // POV Left --> First Stage / Ground intake height
        operatorJoystick.povLeft().onTrue(
            macros.groundIntake()
        );

        // // POV Down --> Stowe
        operatorJoystick.povDown().onTrue(
            macros.stow()
        );

        // // POV Up --> Substration intake height
        operatorJoystick.povUp().onTrue(
            macros.substationIntake()
        );

        operatorJoystick.povRight().onTrue(
            macros.singleSubstationIntake()
        );

        // // X --> Cone 2nd Stage
        operatorJoystick.x().onTrue(
            macros.general2ndStage()
        );

        // // Y --> Cone 3rd Stage
        operatorJoystick.y().onTrue(
            macros.general3rdStage()
        );

        // A --> cone mode
        operatorJoystick.a().onTrue(
            macros.setConeMode()
        );

        // // B --> cube mode
        operatorJoystick.b().onTrue(
            macros.setCubeMode()
        );
    
        // Right Bumper --> Intake 
        operatorJoystick.rightBumper().onTrue(
            macros.intake(lightstrip)
        );

        // // Left Bumper --> Outtake
        operatorJoystick.leftBumper().onTrue(
            macros.outtake()
        );

        // operatorJoystick.rightBumper().onTrue(new IntakePiece(intake));
        // operatorJoystick.leftBumper().onTrue(new ArmCharacterization(newArm, operatorJoystick));

        // Back --> Manual Intake Stop
        operatorJoystick.back().onTrue(Commands.runOnce(() -> intake.stop(), intake));

        // driverJoystick.a().onTrue(Commands.run(() -> swerve.drive(1.00, 0.000, 0.000), swerve));
  }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return m_Chooser.getSelected(); 
        return new exampleAuto(s_Swerve);
    }
}

