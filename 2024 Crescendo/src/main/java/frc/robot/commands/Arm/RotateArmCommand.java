package frc.robot.commands.Arm;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;


public class RotateArmCommand extends Command{

    double setPosition;
    double speed;
    boolean lowering;
    ArmSubsystem armSubsystem = RobotContainer.armSubsystem;
    
    public RotateArmCommand(double setPosition, double speed){
        
        this.setPosition = setPosition;
        this.speed = speed;
        // this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        
        if(armSubsystem.getArmPosition() < setPosition){
            armSubsystem.rotateArm(speed);
            lowering = false;
        }

        else if(armSubsystem.getArmPosition() > setPosition){
            armSubsystem.rotateArm(-speed);
            lowering = true;
        }

        else{
            armSubsystem.StopArm();
        }

    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        armSubsystem.StopArm();
    }

    @Override
    public boolean isFinished() {
        return ((lowering && armSubsystem.getArmPosition() <= setPosition) ||
        !lowering && armSubsystem.getArmPosition() >= setPosition);
    }
}