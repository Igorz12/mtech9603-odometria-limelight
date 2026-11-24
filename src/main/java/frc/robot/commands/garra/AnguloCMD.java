package frc.robot.commands.garra;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.garra.GarraSUB;

public class AnguloCMD extends Command{
    private GarraSUB garraSUB;
    double position;




    public AnguloCMD(GarraSUB garraSUB, double position) {
        this.garraSUB = garraSUB;
        this.position = position;

        addRequirements(garraSUB);
    }
    
    @Override
    public void initialize() {
        garraSUB.setPosition(position);
    }

  
@Override
public boolean isFinished() {
    return Math.abs(garraSUB.getAngulo() - position) < 1.0;
}
}
