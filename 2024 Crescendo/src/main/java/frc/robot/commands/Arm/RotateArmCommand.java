package frc.robot.commands.Arm;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;


public class RotateArmCommand extends Command{

    double motorValue;
    ArmSubsystem armSubsystem;
    CommandJoystick joystick;
    
    public RotateArmCommand(ArmSubsystem armSubsystem, CommandJoystick joystick, double motorValue){
        
        this.joystick = joystick;
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

        else if(armSubsystem.getArmPosition() > Constants.Arm.lowestArmAngle && joystick.getY() > 0.1){
            armSubsystem.rotateArm(motorValue);
        }

        else if(armSubsystem.getArmPosition() > Constants.Arm.lowestArmAngle && joystick.getY() < -0.1){
            armSubsystem.rotateArm(motorValue);
        }

        else{
            armSubsystem.StopArm();
        }

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