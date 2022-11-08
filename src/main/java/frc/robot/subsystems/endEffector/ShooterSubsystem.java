package frc.robot.subsystems.endEffector;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private WPI_TalonSRX shooterMotor;

    public ShooterSubsystem() {
        shooterMotor = new WPI_TalonSRX(Constants.EndEffector.SHOOTER_MOTOR_PORT);

        shooterMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void runShooter() {
        shooterMotor.set(Constants.EndEffector.DEFAULT_SHOOTER_SPEED);
    }

    public void reverseShooter() {
        shooterMotor.set(Constants.EndEffector.DEFAULT_SHOOTER_REVERSE_SPEED);
    }
}

