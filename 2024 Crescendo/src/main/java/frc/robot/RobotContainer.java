// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//WPILIB
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
//COMMANDS
import frc.robot.commands.Autos;
import frc.robot.commands.Arm.ExtendArmCommand;
import frc.robot.commands.Arm.HoldArmCommand;
import frc.robot.commands.Arm.JoystickRotateArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Arm.JoystickRotateArmCommand;
import frc.robot.commands.Arm.PIDRotateArmCommand;
import frc.robot.commands.Arm.RetractArmCommand;
import frc.robot.commands.Arm.RotateArmCommand;
import frc.robot.commands.Arm.JoystickRotateArmCommand;
import frc.robot.commands.Arm.ToggleArmCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.SensorIntakeCommand;
import frc.robot.commandgroups.IntakeNoteCommandGroup;
import frc.robot.commandgroups.IntakeWithArmCommandGroup;
import frc.robot.commandgroups.ManualArmControlCommandGroup;
import frc.robot.commandgroups.RotateAmpCommandGroup;
import frc.robot.commandgroups.RotateSpeakerCommandGroup;
import frc.robot.commandgroups.SpeakerScoreCommandGroup;
import frc.robot.commands.Shooter.AmpScoreCommand;
// import frc.robot.commands.Shooter.InputRunShooterCommand;
import frc.robot.commands.Shooter.PIDShooterCommand;
import frc.robot.commands.Shooter.RunShooterCommand;
// import frc.robot.commands.Shooter.ShooterModeCommand;
//SUBSYSTEMS
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  public final static IntakeShooterSubsystem intakeShooterSubsystem = new IntakeShooterSubsystem();
  public final static ArmSubsystem armSubsystem = new ArmSubsystem();

  private XboxController driverXbox = new XboxController(0);
  public static CommandJoystick shooterJoystick = new CommandJoystick(1);
  PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    setMotorBrake(true);

    // armSubsystem.setDefaultCommand(new PIDRotateArmCommand(armSubsystem.getArmPosition()));
    armSubsystem.setDefaultCommand(new HoldArmCommand(armSubsystem.getArmPosition()));

    // Configure the trigger bindings
    configureBindings();

    // // Applies deadbands and inverts controls because joysticks
    // // are back-right positive while robot
    // // controls are front-left positive
    // // left stick controls translation
    // // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRawAxis(2));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
  }


  private void configureBindings() {
    // shooterJoystick.button(2).onTrue(new ManualArmControlCommandGroup());
    shooterJoystick.button(3).toggleOnTrue(new RetractArmCommand(armSubsystem));
    shooterJoystick.button(7).onTrue(new PIDRotateArmCommand(() -> Constants.Arm.ampArmAngle));
    shooterJoystick.button(8).toggleOnTrue(new AmpScoreCommand(intakeShooterSubsystem));
    shooterJoystick.button(9).onTrue(new PIDRotateArmCommand(() -> Constants.Arm.speakerArmAngle));
    // shooterJoystick.button(10).toggleOnTrue(new PIDShooterCommand(intakeShooterSubsystem, 2000));
    shooterJoystick.button(11).toggleOnTrue(new IntakeWithArmCommandGroup());
    shooterJoystick.button(12).toggleOnTrue(new SensorIntakeCommand(intakeShooterSubsystem, 0.8));
    shooterJoystick.button(1).toggleOnTrue(new IntakeCommand(intakeShooterSubsystem, 0.8));

    // shooterJoystick.button(2).onTrue(new PIDRotateArmCommand(() -> armSubsystem.getArmInput()));
    shooterJoystick.button(2).onTrue(new PIDRotateArmCommand(() -> Constants.Arm.testArmAngle));
    // DoubleSupplier shooterSpeed = () -> intakeShooterSubsystem.getInputShooterSpeed();
    shooterJoystick.button(10).onTrue(new RunShooterCommand(intakeShooterSubsystem, () -> intakeShooterSubsystem.getInputShooterSpeed(), () -> intakeShooterSubsystem.getInputShooterSpeed()));

    new JoystickButton(driverXbox, 2).toggleOnTrue(new InstantCommand(() -> drivebase.zeroGyro()));
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public void trackPowerMetrics() {
    double voltage = powerDistribution.getVoltage();
    double temperatureFahrenheit = (powerDistribution.getTemperature() * 1.8) + 32;
    double totalCurrent = powerDistribution.getTotalCurrent();
    double totalPower = powerDistribution.getTotalPower();
    double totalEnergy = powerDistribution.getTotalEnergy();
    SmartDashboard.putNumber("TotalEnergy", totalEnergy);
    SmartDashboard.putNumber("TotalPower", totalPower);
    SmartDashboard.putNumber("Total Current", totalCurrent);
    SmartDashboard.putNumber("Temperature", temperatureFahrenheit);
    SmartDashboard.putNumber("Voltage", voltage);
    SmartDashboard.putBoolean("Sensor Value", intakeShooterSubsystem.getSensorValue());
  }

  public void joystickValues() {

    SmartDashboard.putNumber("input shooter speed value", intakeShooterSubsystem.getInputShooterSpeed());

    SmartDashboard.putNumber("Y value", driverXbox.getLeftY());
    SmartDashboard.putNumber("X value", driverXbox.getLeftX());
    SmartDashboard.putNumber("Rotation value", driverXbox.getRawAxis(2));
  }

}
