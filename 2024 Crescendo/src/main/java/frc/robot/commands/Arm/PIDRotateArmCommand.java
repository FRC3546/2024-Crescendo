package frc.robot.commands.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class PIDRotateArmCommand extends Command {

    double setPosition;

    ArmSubsystem armSubsystem = RobotContainer.armSubsystem;
    PIDController pidLoop;

    public PIDRotateArmCommand(double setPosition) {

        this.setPosition = setPosition;
        pidLoop = new PIDController(8, 0, 1.3);
        pidLoop.setTolerance(0.05);
        // position here means a value between 0.0 and 1.0 as measured by the motor
        // encoder
        pidLoop.setSetpoint(setPosition);

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        SmartDashboard.putNumber("pidLoop calculating value", pidLoop.calculate(armSubsystem.getArmPosition()));
        armSubsystem.rotateArm(MathUtil.clamp((pidLoop.calculate(armSubsystem.getArmPosition())), -0.5, 0.5));
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.StopArm();
    }

    @Override
    public boolean isFinished() {
        // Should finish when the arm is at the set PID setpoint.
        // return setPosition == armSubsystem.getArmPosition();
        return pidLoop.atSetpoint();
    }
}