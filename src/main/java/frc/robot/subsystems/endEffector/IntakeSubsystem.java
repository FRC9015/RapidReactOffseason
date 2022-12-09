package frc.robot.subsystems.endEffector;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private WPI_TalonSRX intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = new WPI_TalonSRX(Constants.EndEffector.INTAKE_MOTOR_PORT);
        intakeMotor.setInverted(Constants.EndEffector.INTAKE_REVERSED);
        // Intake will not brake stop, it will coast to a stop
        intakeMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void runIntake(double speed) {
        intakeMotor.set(Math.max(-0.8,Math.min((speed),0.8)));
    }

    public void runIntake() {
        intakeMotor.set(Math.max(-0.8,Math.min((Constants.EndEffector.DEFAULT_INTAKE_SPEED),0.8)));
    }

    public void reverseIntake() {
        intakeMotor.set(Math.max(-0.8,Math.min((Constants.EndEffector.DEFAULT_INTAKE_REVERSE_SPEED),0.8)));
    }

    /**
     * Stops the intake motor with coast mode
     */
    public void stopIntake() {
        intakeMotor.set(0);
    }
}

