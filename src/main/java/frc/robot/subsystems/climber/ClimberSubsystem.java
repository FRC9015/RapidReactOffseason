package frc.robot.subsystems.climber;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private WPI_TalonSRX climbMotor;

    public ClimberSubsystem() {
        climbMotor = new WPI_TalonSRX(Constants.Climb.CLIMB_MOTOR_PORT);

        climbMotor.setNeutralMode(NeutralMode.Brake);
    }
}

