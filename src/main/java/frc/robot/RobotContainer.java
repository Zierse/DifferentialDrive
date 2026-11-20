// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SimElevator;
import frc.robot.subsystems.SimHand;
import frc.robot.subsystems.SimIntake;

public class RobotContainer {

  private final TankSubsystem tank = new TankSubsystem();

  // "driver" = CommandPS4Controller (botões sem EventLoop).
  // "driverget" = PS4Controller normal, usado só para ler eixos.
  private final CommandPS4Controller driver = new CommandPS4Controller(0);
  private final PS4Controller driverget = new PS4Controller(0);

  private SimIntake mSimIntake = new SimIntake(20, 1.5, 3);
  private SimElevator mSimElevator = new SimElevator(400, 1.5);
  private SimHand mSimHand = new SimHand(-180, 180, 1.5, 3);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Deadbands (stick do PS4 costuma ter drift em repouso)
  private static final double STICK_THRESHOLD = 0.15;
  private static final double ROTATION_DEADBAND = 0.10;

  // Detecção automática da faixa dos gatilhos (-1..1 ou 0..1)
  private boolean r2Bipolar = false;
  private boolean l2Bipolar = false;

  public RobotContainer() {

    autoChooser.setDefaultOption("Nenhum", Commands.none());
    autoChooser.addOption("Quadrado", Autos.quadrado(tank));
    autoChooser.addOption("Reto", Autos.reto(tank));
    autoChooser.addOption("autoZigZag", Autos.autoZigZag(tank));

    SmartDashboard.putData("Auto Selector", autoChooser);

    configureBindings();

    tank.setDefaultCommand(tank.run(() -> {

      double leftX = driver.getLeftX();
      double leftY = driver.getLeftY();

      double joystickMagnitude = Math.hypot(leftX, leftY);

      double forward = getR2();
      double backward = getL2();

      // Debug: com o controle parado, todos devem ficar em 0
      Logger.recordOutput("PS4/R2", forward);
      Logger.recordOutput("PS4/L2", backward);
      Logger.recordOutput("PS4/RightX", driver.getRightX());
      Logger.recordOutput("PS4/LeftX", leftX);
      Logger.recordOutput("PS4/LeftY", leftY);

      // Joystick esquerdo sendo utilizado
      if (joystickMagnitude > STICK_THRESHOLD) {

        tank.driveWithAngleControl(getLeftJoystickAngle(), forward);

        // Tank normal
      } else {

        double rotation = MathUtil.applyDeadband(driver.getRightX(), ROTATION_DEADBAND);

        tank.drive(forward, -backward, -rotation);
      }
    }));
  }

  private void configureBindings() {

    // Xbox start -> PS4 options
    driver.options().onTrue(tank.runOnce(() -> tank.resetPose(new Pose2d(2.0, 4.0, new Rotation2d()))));

    driver.povUp().onTrue(mSimElevator.CMDsetPosition(400, 10));
    driver.povDown().onTrue(mSimElevator.CMDsetPosition(0, 3));

    driver.povLeft().onTrue(Commands.runOnce(() -> mSimHand.setAngle(-45, 3)));
    driver.povRight().onTrue(Commands.runOnce(() -> mSimHand.setAngle(45, 3)));

    // Xbox A -> PS4 X (cross) | Xbox B -> PS4 bolinha (circle)
    driver.cross().onTrue(Commands.runOnce(() -> mSimHand.setAngle(135, 3)));
    driver.circle().onTrue(Commands.runOnce(() -> mSimHand.setAngle(-135, 3)));

    // Xbox back -> PS4 share
    driver.share().onTrue(Commands.runOnce(() -> mSimHand.setAngle(0, 3)));

    coletar_Ground();
    autoHand();
    // deposit_Pose();

    // Xbox Y -> PS4 triângulo
    driver.triangle().onTrue(tank.depositFuelCommand());

  }

  /** Gatilho direito (R2) normalizado de 0 a 1. */
  private double getR2() {
    double raw = driver.getR2Axis();
    if (raw < -0.9) {
      r2Bipolar = true;
    }
    return MathUtil.clamp(r2Bipolar ? (raw + 1.0) / 2.0 : raw, 0.0, 1.0);
  }

  /** Gatilho esquerdo (L2) normalizado de 0 a 1. */
  private double getL2() {
    double raw = driver.getL2Axis();
    if (raw < -0.9) {
      l2Bipolar = true;
    }
    return MathUtil.clamp(l2Bipolar ? (raw + 1.0) / 2.0 : raw, 0.0, 1.0);
  }

  void coletar_Ground() {

    // Xbox leftBumper -> PS4 L1
    driver.L1().whileTrue(mSimIntake.CMDintakeOn().onlyIf(() -> !tank.isFuelInsideRobot()));
    driver.L1().whileFalse(mSimIntake.CMDintakeOff());

    /*
     * driver.L1().onTrue(
     * mSimElevator.CMDsetPosition(0, 3)
     * .alongWith(Commands.runOnce(() -> mSimHand.setAngle(0, 3))));
     * driver.L1().onFalse(
     * mSimElevator.CMDsetPosition(400, 10)
     * );
     */
  }

  void autoHand() {

    new Trigger(tank::isFuelInsideRobot).onTrue(
        Commands.runOnce(() -> mSimHand.setAngle(0, 3))
            .andThen(Commands.waitSeconds(0.5))
            .andThen(mSimElevator.CMDsetPosition(0, 3))
            .onlyIf(() -> {
              double pos = mSimElevator.getRawPosition();
              boolean mao = tank.isHandCollected();

              Logger.recordOutput("AutoHand/PosNoOnlyIf", pos);
              Logger.recordOutput("AutoHand/MaoOcupadaNoOnlyIf", mao);

              return pos > 0.01 && !mao;
            }));

    // Soltou o L1 -> sobe o elevador
    driver.L1().onFalse(mSimElevator.CMDsetPosition(400, 10));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Auto");
  }

  public double getLeftJoystickAngle() {

    double x = -driverget.getLeftX();
    double y = -driverget.getLeftY();

    if (Math.hypot(x, y) < STICK_THRESHOLD) {
      return 0.0;
    }

    return Math.toDegrees(Math.atan2(x, y));
  }

  /**
   * Ativa o comando alvo, quando a condição é verdadeira.
   * 
   * @param condition condição que dispara o comando
   * @param command   comando a ser executado
   */
  private void onCommandCondition(BooleanSupplier condition, Command command) {
    new Trigger(condition).onTrue(command);
  }

  /**
   * Matém um comando ativo, enquanto a condição é verdade.
   * 
   * @param condition condição que mantém o comando ativo
   * @param command   comando a ser executado
   */
  private void whileCommandCondition(BooleanSupplier condition, Command command) {
    new Trigger(condition).whileTrue(command);
  }

  Command deposit() {
    return Commands.sequence(
        Commands.runOnce(() -> mSimHand.setVelocity(1)),
        tank.depositFuelCommand(),
        Commands.waitSeconds(1),
        Commands.runOnce(() -> mSimHand.setVelocity(0)));
  }

}