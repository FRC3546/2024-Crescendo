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


public class JoystickRotateArmCommand extends Command{

    DoubleSupplier motorValue;
    ArmSubsystem armSubsystem = RobotContainer.armSubsystem;
     //private FlipperSubsystem m_flipperSubsystem = RobotContainer.m_flipperSubsystem;
    
    public JoystickRotateArmCommand(DoubleSupplier motorValue){
        
        this.motorValue = motorValue;
         this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

     @Override
     public void initialize(){
        


     }

    @Override
    public void execute() {

        if(armSubsystem.getArmPosition() > Constants.Arm.lowestArmAngle &&
        armSubsystem.getArmPosition() < Constants.Arm.highestArmAngle){
            armSubsystem.rotateArm((motorValue.getAsDouble() / 4));
            System.out.println("in bounds");
        }

        else if(armSubsystem.getArmPosition() <= Constants.Arm.lowestArmAngle && motorValue.getAsDouble() > 0.1){
            armSubsystem.rotateArm((motorValue.getAsDouble() / 4));
            System.out.println("Going back up into bounds");
        }

        else if(armSubsystem.getArmPosition() >= Constants.Arm.highestArmAngle && motorValue.getAsDouble() < -0.1){
            armSubsystem.rotateArm((motorValue.getAsDouble() / 4));
            System.out.println("Going back down into bounds");

        }

        else{
            armSubsystem.StopArm();
            System.out.println("stopping");
            System.out.println(motorValue);
            System.out.println(armSubsystem.getArmPosition());
        }

         armSubsystem.rotateArm((motorValue.getAsDouble() / 4));

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