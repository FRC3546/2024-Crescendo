package frc.robot.commands.Arm;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;


public class RotateArmCommand extends Command{

    double motorValue;
    ArmSubsystem armSubsystem;
    
    public RotateArmCommand(ArmSubsystem armSubsystem, double motorValue){
        
        this.armSubsystem = armSubsystem;
        this.motorValue = motorValue;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        


    }

    @Override
    public void execute() {

        if(armSubsystem.getArmPosition() < Constants.Arm.lowestArmAngle &&
        armSubsystem.getArmPosition() > Constants.Arm.highestArmAngle){
            armSubsystem.rotateArm(motorValue);
        }

        else if(armSubsystem.getArmPosition() > Constants.Arm.lowestArmAngle){

        }

    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}