package frc.robot.commands.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class PIDRotateArmCommand extends Command {

    DoubleSupplier setPosition;

    ArmSubsystem armSubsystem = RobotContainer.armSubsystem;
    PIDController pidLoop;

    public PIDRotateArmCommand(DoubleSupplier setPosition) {

        this.setPosition = setPosition;
        pidLoop = new PIDController(8, 0, 1.3);
        pidLoop.setTolerance(0.02);
        // position here means a value between 0.0 and 1.0 as measured by the motor
        // encoder
        pidLoop.setSetpoint(setPosition.getAsDouble());

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println(setPosition);
    }

    @Override
    public void execute() {

        SmartDashboard.putNumber("pidLoop calculating value", pidLoop.calculate(armSubsystem.getArmPosition()));
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