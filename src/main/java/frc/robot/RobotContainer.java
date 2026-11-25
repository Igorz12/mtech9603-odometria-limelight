// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.elevator.ElevatorCMD;
import frc.robot.commands.garra.AnguloCMD;
import frc.robot.commands.garra.ColetarCMD;
import frc.robot.commands.garra.EjetarCMD;
import frc.robot.commands.sequential.SequentialL3CMD;
import frc.robot.commands.sequential.SequentialL2CMD;
import frc.robot.subsystems.elevador.ElevatorSUB;
import frc.robot.subsystems.elevador.ElevatorSUB.ElevatorState;
import frc.robot.subsystems.garra.GarraSUB;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.Set;

import swervelib.SwerveInputStream;


public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final          CommandXboxController driverXbox = new CommandXboxController(0);
  
      final CommandXboxController co_piloto = new CommandXboxController(1);
  
       private final ElevatorSUB elevado = new ElevatorSUB();
    private final GarraSUB garra = new GarraSUB();
  
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                  "swerve/neo"));
  
    // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
    private final SendableChooser<Command> autoChooser;
  
    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
     */
    SwerveInputStream driveDirectAngle = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                  () -> driverXbox.getLeftY() * -1,
                                                                  () -> driverXbox.getLeftX() * -1)
                                                              .withControllerHeadingAxis(driverXbox::getRightX,
                                                                                               driverXbox::getRightY)
                                                              .headingWhile(true)
                                                              .deadband(OperatorConstants.DEADBAND)
                                                              .scaleTranslation(0.8)
                                                              .allianceRelativeControl(true);
  
  
   // Converte driver para movimentação de Angulação controlada
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> -driverXbox.getLeftY() * 0.7, // Alterar a força de translação Y do robô
        () -> -driverXbox.getLeftX() * 0.7) // Alterar a força de translação X do robô
        .withControllerRotationAxis(() -> -driverXbox.getRightX() * 0.7) // Altarar a força de rotação do robô
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
      // Configure the trigger bindings
      configureBindings();
      DriverStation.silenceJoystickConnectionWarning(true);
      
      //Create the NamedCommands that will be used in PathPlanner
      NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  
      //Have the autoChooser pull in all PathPlanner autos as options
      autoChooser = AutoBuilder.buildAutoChooser();
      
      //Put the autoChooser on the SmartDashboard
      SmartDashboard.putData("Auto Chooser", autoChooser);
    }
  
  
    private void configureBindings()
    {
      Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
  

      driverXbox.rightBumper().onTrue(drivebase.runOnce(drivebase::zeroDriverHeading));

       driverXbox.x().onTrue(Commands.defer(() -> drivebase.driveToAlliancePose(
        Constants.FieldPose.pose1, 5, Units.degreesToRadians(0)),Set.of(drivebase)));
  
      driverXbox.leftBumper().onTrue(Comandos.ElevatorCMD(elevado, ElevatorState.L3));
      // driverXbox.x().onTrue(Comandos.ElevatorCMD(elevado, ElevatorState.L1));
      driverXbox.a().onTrue(Comandos.ElevatorCMD(elevado, ElevatorState.ZERO));
      driverXbox.b().onTrue(Comandos.SequentialL2CMD(elevado, garra));
      driverXbox.y().onTrue(Comandos.SequentialL3CMD(elevado, garra));
  
      driverXbox.start().onTrue(Comandos.AnguloCMD(garra, Constants.GarraSUBConstants.ANGULO_GARRA));
      driverXbox.back().onTrue(Comandos.AnguloCMD(garra, Constants.GarraSUBConstants.ANGULO_ZERO));
  
         driverXbox.rightTrigger().toggleOnTrue(Comandos.EjetarCMD(garra, Constants.GarraSUBConstants.VELOCIDADE_EJECAO));
    driverXbox.leftTrigger().toggleOnTrue(Comandos.ColetarCMD(garra, Constants.GarraSUBConstants.VELOCIDADE_COLETA));
    }
  
  
     // Commands
    public static class Comandos {
  
      public static Command ElevatorCMD(ElevatorSUB elevatorSUB, ElevatorState nivel) {
        return new ElevatorCMD(elevatorSUB, nivel);
      }
  
      public static Command ColetarCMD(GarraSUB garra, double rpm) {
        return new ColetarCMD(garra, rpm);
      }
  
      public static Command EjetarCMD(GarraSUB garra, double rpm) {
        return new EjetarCMD(garra, rpm);
      }
  
      public static Command SequentialL3CMD(ElevatorSUB elevatorSUB, GarraSUB garraSUB) {
        return new SequentialL3CMD(elevatorSUB, garraSUB);
  
      }
  
      public static Command SequentialL2CMD(ElevatorSUB elevatorSUB, GarraSUB garraSUB) {
        return new SequentialL2CMD(elevatorSUB, garraSUB);
      }
  
      public static Command AnguloCMD(GarraSUB garra, double angle) {
        return new AnguloCMD(garra, angle);
      }
    }
  
     
   /** True se o piloto mexeu em algum joystick. */


  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

    public Command alignSwerveWithAngle(double angleInDegrees) {
    Rotation2d rotationToAlign = Rotation2d.fromDegrees(angleInDegrees);
    SwerveInputStream alignWithAngles = driveDirectAngle.copy().withControllerHeadingAxis(
      rotationToAlign::getCos, 
      rotationToAlign::getSin);
      return drivebase.driveFieldOriented(alignWithAngles);
  }
}
