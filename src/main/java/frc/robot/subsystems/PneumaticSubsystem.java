package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PneumaticSubsystem implements Subsystem {
    private PneumaticsControlModule pcm;
    private Compressor compressor;

    public PneumaticSubsystem() {
        pcm = new PneumaticsControlModule(0);
        compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    }

    public void setSolenoid(int solenoid, boolean state) {
    }

}

