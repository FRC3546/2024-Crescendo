package frc.robot.commands.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class AutoTargetArmCommand extends Command {


    ArmSubsystem armSubsystem;
    LimelightSubsystem limelightSubsystem;
    PIDController pidLoop;

    public AutoTargetArmCommand(ArmSubsystem armSubsystem, LimelightSubsystem limelightSubsystem) {

        pidLoop = new PIDController(8, 0, 1.3);
        // position here means a value between 0.0 and 1.0 as measured by the motor
        // encoder
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        pidLoop.setTolerance(0.02);
        double limelightY = limelightSubsystem.getLimelightY();
        double armAngle = -0.00008 * Math.pow(limelightY,2) + .00252*limelightY + .4992;
        pidLoop.setSetpoint(armAngle);

        SmartDashboard.putNumber("limelight calculated arm angle", armAngle);
    }

    @Override
    public void execute() {
        armSubsystem.rotateArm(MathUtil.clamp((pidLoop.calculate(armSubsystem.getArmPosition())), -1, 1));
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.StopArm();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}