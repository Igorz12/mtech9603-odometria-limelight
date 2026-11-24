package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevador.ElevatorSUB;
import frc.robot.subsystems.elevador.ElevatorSUB.ElevatorState;

public class ElevatorCMD extends Command {

    private final ElevatorSUB elevador;
    private final ElevatorState nivel;

    public ElevatorCMD(ElevatorSUB elevador, ElevatorState nivel) {
        this.elevador = elevador;
        this.nivel = nivel;
        addRequirements(elevador);
    }

    @Override
    public void initialize() {
        elevador.setNivel(nivel);
    }

    @Override
    public boolean isFinished() {
        return elevador.noAlvo();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevador.manterPosicaoAtual();
        }
    }
}