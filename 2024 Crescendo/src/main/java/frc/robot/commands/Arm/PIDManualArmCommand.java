package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class PIDManualArmCommand extends Command {

    double setPosition;

    PIDController pidLoop;
    ArmSubsystem armSubsystem = RobotContainer.armSubsystem;

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        
        SmartDashboard.putNumber("pidLoop calculating value", pidLoop.calculate(armSubsystem.getArmPosition()));
        armSubsystem.rotateArm(MathUtil.clamp((pidLoop.calculate(armSubsystem.getArmPosition())), -0.5, 0.5));

    }
}