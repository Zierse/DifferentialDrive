// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

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

  // IMPORTANTE: o "driver" precisa ser CommandPS4Controller (botões sem EventLoop).
  // O "driverget" é o PS4Controller normal, usado só para ler eixos.
  private final CommandPS4Controller driver = new CommandPS4Controller(0);
  private final PS4Controller driverget = new PS4Controller(0);

  private SimIntake mSimIntake = new SimIntake(20, 1.5, 3);
  private SimElevator mSimElevator = new SimElevator(400, 1.5);
  private SimHand mSimHand = new SimHand(-180, 180, 1.5, 3);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Deadbands (stick do PS4 costuma ter drift em repouso)
  private static final double STICK_THRESHOLD = 0.15;
  private static final double ROTATION_DEADBAND = 0.10;

  public RobotContainer() {

    autoChooser.setDefaultOption("Nenhum", Commands.none());
    autoChooser.addOption("Quadrado", Autos.quadrado(tank));
    autoChooser.addOption("Reto", Autos.reto(tank));
    autoChooser.addOption("autoZigZag", Autos.autoZigZag(tank));
    // autoChooser.addOption("Algae_3", Autos.Algae_3(tank));

    SmartDashboard.putData("Auto Selector", autoChooser);

    configureBindings();

    tank.setDefaultCommand(tank.run(() -> {

      double leftX = driver.getLeftX();
      double leftY = driver.getLeftY();

      double joystickMagnitude = Math.hypot(leftX, leftY);

      double forward = getR2();
      double backward = getL2();

      // Debug: confira estes valores no AdvantageScope com o controle parado
      SmartDashboard.putNumber("PS4/R2", forward);
      SmartDashboard.putNumber("PS4/L2", backward);
      SmartDashboard.putNumber("PS4/RightX", driver.getRightX());
      SmartDashboard.putNumber("PS4/LeftX", leftX);
      SmartDashboard.putNumber("PS4/LeftY", leftY);

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

    /*
     * Positivo = Depositar
     * Negativo = Coletar
     */
    coletar_Ground();
    autoHand();
    // deposit_Pose();

    // Xbox Y -> PS4 triângulo
    driver.triangle().onTrue(deposit());

  }

  // Detecção automática da faixa dos gatilhos:
  // - Controle real/DS: repouso em -1 (faixa -1..1)
  // - Controle não detectado ou faixa 0..1: repouso em 0
  // Assim, controle desconectado = robô parado (e não "frente - ré = 0.5 - 0.5").
  private boolean r2Bipolar = false;
  private boolean l2Bipolar = false;

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
    driver.L1().whileTrue(
        mSimIntake.CMDintakeOn().onlyIf(() -> !tank.isFuelInsideRobot() && SimElevator.getRealPosition() >= 200));
    driver.L1().whileFalse(mSimIntake.CMDintakeOff());
  }

  void autoHand() {

    BooleanSupplier autoHandOk = () -> tank.isFuelInsideRobot() && !tank.isHandCollected();

    onCommandCondition(autoHandOk, Commands.sequence(
        mSimElevator.CMDsetPosition(400, 3),
        Commands.waitUntil(() -> SimElevator.getRealPosition() > 390),
        mSimHand.CMDsetAngle(0, 3),
        Commands.waitUntil(() -> Math.abs(SimHand.getRealAngle()) < 5),
        mSimElevator.CMDsetPosition(0, 3),
        mSimHand.CMDsetVelocity(-1),
        Commands.waitSeconds(1),
        mSimHand.CMDsetVelocity(0)));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
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
