// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.auto.AutoSequenceCommandGroup;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DiffDriveSubsystem;
import frc.robot.subsystems.endEffector.IntakeHeightSubsystem;
import frc.robot.subsystems.endEffector.IntakeSubsystem;
import frc.robot.subsystems.endEffector.ShooterSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer


{
    private final static RobotContainer INSTANCE = new RobotContainer();

    /**
     * Returns the Singleton instance of this RobotContainer. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class.
     */
    @SuppressWarnings("WeakerAccess")
    public static RobotContainer getInstance() {
        return INSTANCE;
    }
    // The robot's subsystems and commands are defined here...

    public final DiffDriveSubsystem diffDriveSubsystem = new DiffDriveSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final IntakeHeightSubsystem intakeHeightSubsystem = new IntakeHeightSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final Command autoCommand = new AutoSequenceCommandGroup(diffDriveSubsystem, intakeSubsystem, shooterSubsystem);
    private final DriveCommand driveCommand = new DriveCommand(diffDriveSubsystem);
    private final ClimbCommand climbCommand = new ClimbCommand(climberSubsystem);
    private final PhotonVisionSubsystem photonVisionSubsystem = new PhotonVisionSubsystem();
   //private final HomingCommand homingCommand = new HomingCommand(photonVisionSubsystem, diffDriveSubsystem);
    //private final homing2 homing2 = new homing2(diffDriveSubsystem, photonVisionSubsystem);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer()
    {
        // Configure the button bindings
        configureButtonBindings();
        
        diffDriveSubsystem.setDefaultCommand(driveCommand);
        climberSubsystem.setDefaultCommand(climbCommand);
    }
    
    
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        // Add button to command mappings here.
        // See https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html

        JoystickButton driveXbutton = new JoystickButton(getDriverJoystick(), XboxController.Button.kX.value);
        driveXbutton.whileHeld(new homing2(diffDriveSubsystem, photonVisionSubsystem));

        // While A is held, run only Intake slowly
        // JoystickButton opAButton = new JoystickButton(getOperatorJoystick(), XboxController.Button.kA.value);
        JoystickButton driveAbutton = new JoystickButton(getDriverJoystick(), XboxController.Button.kA.value);
        //driveAbutton.whileHeld(new fwdshootercmd(diffDriveSubsystem, shooterSubsystem));

        JoystickButton driveYButton = new JoystickButton(getDriverJoystick(), XboxController.Button.kY.value);
        //driveAbutton.whileActiveContinuous(new IntakeOneCommand(intakeSubsystem));
        driveYButton.whileActiveContinuous(new IntakeOneCommand(intakeSubsystem));

        // While A and X are held, run Intake and Shooter slowly
        JoystickButton opXButton = new JoystickButton(getOperatorJoystick(), XboxController.Button.kX.value);
        JoystickButton driveLeftBumper = new JoystickButton(getDriverJoystick(), XboxController.Button.kLeftBumper.value);
        driveLeftBumper.whileActiveContinuous(new ShooterCommand(shooterSubsystem));
        opXButton.and(driveAbutton).whileActiveContinuous(new IntakeCommand(intakeSubsystem, shooterSubsystem));

        // While B is held, run Intake and Shooter slowly in reverse
        JoystickButton opBButton = new JoystickButton(getOperatorJoystick(), XboxController.Button.kB.value);
        opBButton.whileHeld(new ReverseIntakeCommand(intakeSubsystem, shooterSubsystem));

        // While X is held and A is not, run Shooter at full speed
        opXButton.and(driveAbutton.negate()).whileActiveContinuous(new ShooterCommand(shooterSubsystem));

        // When the right bumper is pressed, jerk the robot
        JoystickButton driveRightBumper = new JoystickButton(getDriverJoystick(), XboxController.Button.kRightBumper.value);
        //driveRightBumper.whenPressed(new spinny(diffDriveSubsystem).withTimeout(5.0));

        JoystickButton driveBbutton = new JoystickButton(getDriverJoystick(), XboxController.Button.kB.value);
   
        driveAbutton.whileActiveContinuous(new IntakeHeightDownCommand(intakeHeightSubsystem));
        driveBbutton.whileActiveContinuous(new IntakeHeightUpCommand(intakeHeightSubsystem));

        JoystickButton opLeftBumper = new JoystickButton(getOperatorJoystick(), XboxController.Button.kLeftBumper.value);
        opLeftBumper.whenPressed(new JerkFwdCommand(diffDriveSubsystem).withTimeout(0.09));
        opLeftBumper.whenPressed(new IntakeOneCommand(intakeSubsystem, true).withTimeout(1.0));

        JoystickButton driveStart = new JoystickButton(getDriverJoystick(), XboxController.Button.kStart.value);
        JoystickButton driveBack = new JoystickButton(getDriverJoystick(), XboxController.Button.kBack.value);

        driveStart.whileActiveContinuous(new ClimbUpCommand(climberSubsystem));
        driveBack.whileActiveContinuous(new ClimbDownCommand(climberSubsystem));
    }

    public XboxController getDriverJoystick() {
        return new XboxController(0);
    }


    public XboxController getOperatorJoystick() {
        return new XboxController(1);
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return autoCommand;
    }

    public Command getTeleopCommand() {
        return driveCommand;
    }
}
