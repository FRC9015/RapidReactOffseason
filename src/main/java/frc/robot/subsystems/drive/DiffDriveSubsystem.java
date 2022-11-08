package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DiffDriveSubsystem extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this DiffDriveSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static DiffDriveSubsystem INSTANCE = new DiffDriveSubsystem();

    /**
     * Returns the Singleton instance of this DiffDriveSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code DiffDriveSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static DiffDriveSubsystem getInstance() {
        return INSTANCE;
    }

    public Constants.Drive.DifferentialControlScheme differentialControlScheme = Constants.Drive.DifferentialControlScheme.TANK;
    private final MotorControllerGroup left;
    private final WPI_TalonSRX left1;
    private final WPI_TalonSRX left2;
    private final MotorControllerGroup right;
    private final WPI_TalonSRX right1;
    private final WPI_TalonSRX right2;
    private final DifferentialDrive drive;

    /**
     * Creates a new instance of this DiffDriveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private DiffDriveSubsystem() {
        left1 = new WPI_TalonSRX(Constants.Drive.LEFT_FRONT_MOTOR_PORT);
        addChild("motor_left1", left1);
        left2 = new WPI_TalonSRX(Constants.Drive.LEFT_BACK_MOTOR_PORT);
        addChild("motor_left2", left2);
        left = new MotorControllerGroup(left1, left2);
        addChild("motorGroup_left", left);

        right1 = new WPI_TalonSRX(Constants.Drive.RIGHT_FRONT_MOTOR_PORT);
        addChild("motor_right1", right1);
        right2 = new WPI_TalonSRX(Constants.Drive.RIGHT_BACK_MOTOR_PORT);
        addChild("motor_right2", right2);
        right = new MotorControllerGroup(right1, right2);
        addChild("motorGroup_right", right);

        // Properly invert motors
        left.setInverted(Constants.Drive.LEFT_DRIVE_INVERTED);
        right.setInverted(Constants.Drive.RIGHT_DRIVE_INVERTED);

        // Instantiate the drive class
        drive = new DifferentialDrive(left, right);
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        super.setDefaultCommand(defaultCommand);
    }

    public void drive(double left, double right) {
        if (differentialControlScheme == Constants.Drive.DifferentialControlScheme.TANK) {
            drive.tankDrive(left, right);
            System.out.println("Tank Drive");
            System.out.println("Left: "+ left);
            System.out.println("Right: "+ right);
        } else if (differentialControlScheme == Constants.Drive.DifferentialControlScheme.ARCADE) {
            drive.arcadeDrive(left, right);
            System.out.println("Arcade Drive");
            System.out.println("Fwd: "+ left);
            System.out.println("Rot: "+ right);
        }
    }

    public void arcadeDrive(double fwd, double turn) {
        drive.arcadeDrive(fwd, turn);
        System.out.println("Arcade Drive");
        System.out.println("Fwd: "+ fwd);
        System.out.println("Rot: "+ turn);
    }

    public void tankDrive(double left, double right) {
        drive.tankDrive(left, right);
        System.out.println("Tank Drive");
        System.out.println("Left: "+ left);
        System.out.println("Right: "+ right);
    }
}

