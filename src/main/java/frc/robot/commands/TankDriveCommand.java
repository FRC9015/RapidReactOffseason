package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DiffDriveSubsystem;
public class TankDriveCommand extends CommandBase {
    private final DiffDriveSubsystem diffDriveSubsystem;

    public TankDriveCommand(DiffDriveSubsystem diffDriveSubsystem) {
        this.diffDriveSubsystem = diffDriveSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.diffDriveSubsystem);

    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer robot = RobotContainer.getInstance();
        diffDriveSubsystem.tankDrive(robot.getDriverJoystick().getLeftY(), robot.getDriverJoystick().getRightY());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        diffDriveSubsystem.drive(0.0, 0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
