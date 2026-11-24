package frc.robot.subsystems.elevador;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorSUBConstants;

public class ElevatorSUB extends SubsystemBase {

    private final SparkMax Motor1 = new SparkMax(ElevatorSUBConstants.MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = Motor1.getEncoder();
    private final SparkMaxConfig configMotor1 = new SparkMaxConfig();
    private final SparkClosedLoopController closed = Motor1.getClosedLoopController(); // envia setpoints

    private final Debouncer noAlvoDebouncer = new Debouncer(0.2, DebounceType.kRising);

    // Cada estado já guarda a sua própria posição
    public enum ElevatorState {
        ZERO(ElevatorSUBConstants.point0),
        L1(ElevatorSUBConstants.pointL1),
        L2(ElevatorSUBConstants.pointL2),
        L3(ElevatorSUBConstants.pointL3);

        public final double posicao;

        ElevatorState(double posicao) {
            this.posicao = posicao;
        }
    }

    private ElevatorState estado = ElevatorState.ZERO; // null = parado fora de um nível (ver manterPosicaoAtual)
    private double posicaoAlvo = 0.0;
    private boolean noAlvo = false;
    private static final double TOLERANCIA = 1.0;

    public ElevatorSUB() {

        // Closed Loop
        configMotor1.closedLoop.pid(0.08, 0.0, 0.0);
        configMotor1.closedLoop.outputRange(-0.23, 0.4);
        configMotor1.closedLoopRampRate(0.7);
        configMotor1.closedLoop.feedForward.kG(0.9);
        configMotor1.closedLoop.feedForward.kS(0.0);

        // Configs padrões
        configMotor1.smartCurrentLimit(50);
        configMotor1.idleMode(IdleMode.kBrake);
        configMotor1.inverted(false);
        configMotor1.encoder.positionConversionFactor(1.0);

        Motor1.configure(configMotor1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder.setPosition(0);
    }

    // ==== GETTERS ====
    public double getPosicao() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public ElevatorState getEstado() {
        return estado;
    }

    public boolean noAlvo() {
        return noAlvo;
    }

    // ==== SETTERS ====
    public void setNivel(ElevatorState nivel) {
        estado = nivel;
        irPara(nivel.posicao);
    }

    public void manterPosicaoAtual() {
        estado = null; // não está em nenhum nível
        irPara(getPosicao());
    }

    private void irPara(double posicao) {
        posicaoAlvo = MathUtil.clamp(posicao,
                ElevatorSUBConstants.min_position,
                ElevatorSUBConstants.max_position);

        noAlvoDebouncer.calculate(false); // começa a contagem do zero
        noAlvo = false;
        closed.setSetpoint(posicaoAlvo, ControlType.kPosition);
    }

    // ==== PERIODIC ====
    @Override
    public void periodic() {
        boolean pertoDoAlvo = Math.abs(getPosicao() - posicaoAlvo) < TOLERANCIA;
        noAlvo = noAlvoDebouncer.calculate(pertoDoAlvo);

        Logger.recordOutput("Elevator/Posicao", getPosicao());
        Logger.recordOutput("Elevator/PosicaoAlvo", posicaoAlvo);
        Logger.recordOutput("Elevator/Velocidade", getVelocity());
        Logger.recordOutput("Elevator/Output", Motor1.getAppliedOutput());
        Logger.recordOutput("Elevator/Estado", estado == null ? "MANUAL" : estado.name());
        Logger.recordOutput("Elevator/NoAlvo", noAlvo);
    }
}