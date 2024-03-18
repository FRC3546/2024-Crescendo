// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

//WPILIB
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Auto.AutoLeave;
import frc.robot.Auto.OneNoteLoadSideAuto;
import frc.robot.Auto.TimedDrive;
import frc.robot.Auto.TwoNoteAutoBlue;
import frc.robot.Auto.TwoNoteAutoRed;
import frc.robot.Constants.OperatorConstants;
//COMMANDS

import frc.robot.commands.Arm.HoldArmCommand;
import frc.robot.commands.Limelight.TargetClimbing;
import frc.robot.commands.Limelight.TargetClimbing;
import frc.robot.commands.Arm.JoystickRotateArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

import frc.robot.commands.Arm.PIDRotateArmCommand;

import frc.robot.commands.Climb.JoystickClimbCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.ReverseIntakeCommand;
import frc.robot.commands.Limelight.TargetOnTheMove;
import frc.robot.commandgroups.JoystickActions.IntakeButton;
import frc.robot.commandgroups.JoystickActions.StowedButton;
import frc.robot.commandgroups.JoystickActions.AmpButton;
import frc.robot.commandgroups.JoystickActions.CloseSpeakerButton;
import frc.robot.commandgroups.JoystickActions.FarSpeakerButton;
// import frc.robot.commands.Shooter.InputRunShooterCommand;
import frc.robot.commands.Shooter.RunShooterCommand;
import frc.robot.commands.Shooter.TimedRunShooterCommand;
// import frc.robot.commands.Shooter.ShooterModeCommand;
//SUBSYSTEMS
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

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

  SendableChooser<Command> autos = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  public final static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final static ArmSubsystem armSubsystem = new ArmSubsystem();
  public final static ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  public final static LedSubsystem ledSubsystem = new LedSubsystem();
  
  

  //JOYSTICKS
  private XboxController driverXbox = new XboxController(0);
  public static CommandJoystick shooterJoystick = new CommandJoystick(1);
  private CommandJoystick climberJoystick = new CommandJoystick(2);

  PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {


    autos.addOption("Backup Auto", new TimedDrive(drivebase, 1.5, 0, 0, 2));
    autos.addOption("One Note Auto", new OneNoteLoadSideAuto(drivebase, intakeSubsystem, shooterSubsystem, armSubsystem));
    autos.addOption("BLUE 2 Note Auto", new TwoNoteAutoBlue(drivebase, intakeSubsystem, shooterSubsystem, ledSubsystem, armSubsystem));
    autos.addOption("RED 2 Note Auto", new TwoNoteAutoRed(drivebase, intakeSubsystem, shooterSubsystem, ledSubsystem, armSubsystem));
    autos.addOption("pathplanner test", new ParallelCommandGroup(drivebase.getAutonomousCommand("Test Path", true), new RunShooterCommand(shooterSubsystem, () -> 0, () -> 0)));
    SmartDashboard.putData("Autonomous", autos);

    armSubsystem.setDefaultCommand(new HoldArmCommand(armSubsystem.getArmPosition()));

    configureBindings();
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> driverXbox.getRawAxis(1),
        () -> -driverXbox.getRawAxis(0),
        () -> -driverXbox.getRawAxis(2));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

    shooterSubsystem.setDefaultCommand(new RunShooterCommand(shooterSubsystem,  () -> 0.6,  () -> 0.6));
  }


  private void configureBindings() {

    //CLIMBER
    climberJoystick.button(1).whileTrue(new JoystickClimbCommand(climbSubsystem, () -> climberJoystick.getY()));
    climberJoystick.button(3).toggleOnTrue(new InstantCommand(() -> climbSubsystem.retractClimberPiston()));
    climberJoystick.button(2).toggleOnTrue(new InstantCommand(() -> climbSubsystem.extendClimberPiston()));
    climberJoystick.button(11).toggleOnTrue(new TargetClimbing(limelightSubsystem, drivebase, () -> -2.13 ));
    // climberJoystick.button(6).onTrue(new InstantCommand(() -> ledSubsystem.blue()));
    // climberJoystick.button(7).onTrue(new InstantCommand(() -> ledSubsystem.green()));
    climberJoystick.button(7).whileTrue(new ParallelDeadlineGroup(new ReverseIntakeCommand(intakeSubsystem, -0.5), new TimedRunShooterCommand(shooterSubsystem, () -> (-0.3), () -> (-0.3), 3)));


    //SHOOTER, INTAKE AND ARM CONTROL
    shooterJoystick.button(1).whileTrue(new IntakeCommand(intakeSubsystem, 1));
    shooterJoystick.button(2).onTrue(new PIDRotateArmCommand(() -> Constants.Arm.startingConfigArmAngle));
    
    shooterJoystick.button(5).toggleOnTrue(new JoystickRotateArmCommand(() -> shooterJoystick.getY()));
    shooterJoystick.button(3).whileTrue(new ParallelDeadlineGroup(new ReverseIntakeCommand(intakeSubsystem, -1), new TimedRunShooterCommand(shooterSubsystem, () -> (-0.3), () -> (-0.3), 2)));

    shooterJoystick.button(4).toggleOnTrue(new InstantCommand(() -> armSubsystem.extendArm()));
    shooterJoystick.button(6).toggleOnTrue(new InstantCommand(() -> armSubsystem.retractArm()));
    
    shooterJoystick.button(7).onTrue(new AmpButton(shooterSubsystem, armSubsystem));
    shooterJoystick.button(9).onTrue(new CloseSpeakerButton(armSubsystem, shooterSubsystem));
    shooterJoystick.button(11).toggleOnTrue(new IntakeButton(shooterSubsystem, armSubsystem, intakeSubsystem, ledSubsystem, () -> armSubsystem.getArmExtension()));

    shooterJoystick.button(8).onTrue(new StowedButton(shooterSubsystem, armSubsystem));
    shooterJoystick.button(10).toggleOnTrue(new RunShooterCommand(shooterSubsystem, () -> 0, () -> 0));
    shooterJoystick.button(12).onTrue(new FarSpeakerButton(armSubsystem, shooterSubsystem));    

    //DRIVER CONTROLLER
    new JoystickButton(driverXbox, 1).whileTrue(new TargetOnTheMove(limelightSubsystem, drivebase, () -> (driverXbox.getRawAxis(1)), () -> (-driverXbox.getRawAxis(0)), () -> -2.13));
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
    return autos.getSelected();
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
    SmartDashboard.putNumberArray("botpose", limelightSubsystem.getBotPose());
    SmartDashboard.putBoolean("Sensor Value", intakeSubsystem.getSensorValue());
  }

  public void robotSystemValues(){
    SmartDashboard.putBoolean("climber limit switch value", climbSubsystem.getLimitSwitchValue());
  //   SmartDashboard.putNumber("intake motor speed", intakeSubsystem.getIntakeSpeed());
  //   SmartDashboard.putNumber("input shooter speed value", shooterSubsystem.getInputShooterSpeed());
  //   SmartDashboard.putNumber("left climber motor", climbSubsystem.getLeftEncoder());
  //   SmartDashboard.putNumber("right climber motor", climbSubsystem.getRightEncoder());
  }

  public void joystickValues() {
    SmartDashboard.putNumber("Y value", driverXbox.getLeftY());
    SmartDashboard.putNumber("X value", driverXbox.getLeftX());
    SmartDashboard.putNumber("Rotation value", driverXbox.getRawAxis(2));
  }

  public void limelightValues(){
    SmartDashboard.putNumber("limelight ty", limelightSubsystem.getLimelightY());
    SmartDashboard.putNumber("limelight tx", limelightSubsystem.getLimelightX());
    SmartDashboard.putBoolean("limelight in range", limelightSubsystem.isLimelightXRange());
  }
}
