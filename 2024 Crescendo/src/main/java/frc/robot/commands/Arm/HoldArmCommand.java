package frc.robot.commands.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class HoldArmCommand extends Command {

    double setPosition;

    ArmSubsystem armSubsystem = RobotContainer.armSubsystem;
    PIDController pidLoop;

    public HoldArmCommand(double setPosition) {

        this.setPosition = setPosition;
        pidLoop = new PIDController(8, 0, 1.3);
        pidLoop.setTolerance(0.05);
        // position here means a value between 0.0 and 1.0 as measured by the motor
        // encoder
        // pidLoop.setSetpoint(setPosition);

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

        pidLoop.setSetpoint(setPosition);
        armSubsystem.rotateArm(MathUtil.clamp((pidLoop.calculate(armSubsystem.getArmPosition())), -0.5, 0.5));
    }

    @Override
    public void execute() {

        


        if(!pidLoop.atSetpoint()){
            setPosition = armSubsystem.getArmPosition();
            pidLoop.setSetpoint(setPosition);

          System.out.println(pidLoop.getSetpoint());
    
        
        }
        

        armSubsystem.rotateArm(MathUtil.clamp((pidLoop.calculate(armSubsystem.getArmPosition())), -0.5, 0.5));
        SmartDashboard.putNumber("pidLoop calculating value", pidLoop.calculate(armSubsystem.getArmPosition()));
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("holdarmcommand ended");
        armSubsystem.StopArm();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}