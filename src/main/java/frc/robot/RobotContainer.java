// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
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
    private final Command autoCommand = new AutonomousCommand(diffDriveSubsystem, climberSubsystem, intakeSubsystem, shooterSubsystem);
    private final TankDriveCommand driveCommand = new TankDriveCommand(diffDriveSubsystem);
    private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);

    private final ClimbCommand climbCommand = new ClimbCommand(climberSubsystem);
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private RobotContainer()
    {
        // Configure the button bindings
        configureButtonBindings();
        
        diffDriveSubsystem.setDefaultCommand(driveCommand);
        intakeSubsystem.setDefaultCommand(intakeCommand);
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

        JoystickButton intakeReverseButton = new JoystickButton(getDriverJoystick(), XboxController.Button.kLeftBumper.value);
        intakeReverseButton.whileHeld(new ReverseIntakeCommand(intakeSubsystem));

        JoystickButton shooterActiveButton = new JoystickButton(getDriverJoystick(), XboxController.Button.kX.value);
        shooterActiveButton.and(new JoystickButton(getDriverJoystick(), XboxController.Button.kA.value));

        JoystickButton shooterReverseButton = new JoystickButton(getDriverJoystick(), XboxController.Button.kB.value);
        shooterReverseButton.whileHeld(new ReverseShooterCommand(shooterSubsystem));



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
