package frc.robot.commands.Arm;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;


public class PIDRotateArmCommand extends Command{

    double setPosition;
    ArmSubsystem armSubsystem = RobotContainer.armSubsystem;
    PIDController pidLoop;

    
    public PIDRotateArmCommand(double setPosition){
        
        this.setPosition = setPosition;
        pidLoop = new PIDController(0, 0, 0);
        pidLoop.setTolerance(0.05);
        pidLoop.setSetpoint(setPosition);

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute() {

        SmartDashboard.putNumber("pidLoop calculating value", setPosition);
        armSubsystem.rotateArm(MathUtil.clamp((pidLoop.calculate(armSubsystem.getArmPosition())), -0.25, 0.25));
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