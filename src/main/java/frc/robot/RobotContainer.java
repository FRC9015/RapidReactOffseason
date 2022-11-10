// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.auto.AutoSequenceCommandGroup;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DiffDriveSubsystem;
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
     * Returns the Singleton instance of this DiffDriveSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code DiffDriveSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static RobotContainer getInstance() {
        return INSTANCE;
    }
    // The robot's subsystems and commands are defined here...

    public final DiffDriveSubsystem diffDriveSubsystem = new DiffDriveSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final Command autoCommand = new AutoSequenceCommandGroup(diffDriveSubsystem, intakeSubsystem, shooterSubsystem);
    private final DriveCommand driveCommand = new DriveCommand(diffDriveSubsystem);
    private final ClimbCommand climbCommand = new ClimbCommand(climberSubsystem);
    
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

        // While A is held, run only Intake slowly
        JoystickButton opAButton = new JoystickButton(getOperatorJoystick(), XboxController.Button.kA.value);
        opAButton.whileActiveContinuous(new IntakeOneCommand(intakeSubsystem));

        // While A and X are held, run Intake and Shooter slowly
        JoystickButton opXButton = new JoystickButton(getOperatorJoystick(), XboxController.Button.kX.value);
        opXButton.and(opAButton).whileActiveContinuous(new IntakeCommand(intakeSubsystem, shooterSubsystem));

        // While B is held, run Intake and Shooter slowly in reverse
        JoystickButton opBButton = new JoystickButton(getOperatorJoystick(), XboxController.Button.kB.value);
        opBButton.whileHeld(new ReverseIntakeCommand(intakeSubsystem, shooterSubsystem));

        // While X is held and A is not, run Shooter at full speed
        opXButton.and(opAButton.negate()).whileActiveContinuous(new ShooterCommand(shooterSubsystem));

        // When the right bumper is pressed, jerk the robot
        JoystickButton opRightBumper = new JoystickButton(getOperatorJoystick(), XboxController.Button.kRightBumper.value);
        opRightBumper.whenPressed(new JerkCommand(diffDriveSubsystem).withTimeout(0.055));
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
