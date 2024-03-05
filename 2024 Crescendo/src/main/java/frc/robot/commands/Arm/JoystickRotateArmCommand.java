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
        addRequirements(armSubsystem);
    }

     @Override
     public void initialize(){
        


     }

    @Override
    public void execute() {

        if(armSubsystem.getArmPosition() > Constants.Arm.intakeArmAngle &&
        armSubsystem.getArmPosition() < Constants.Arm.ampArmAngle){
            armSubsystem.rotateArm((motorValue.getAsDouble() / 4));
            // System.out.println("in bounds");
        }

        else if(armSubsystem.getArmPosition() <= Constants.Arm.intakeArmAngle && motorValue.getAsDouble() > 0.25){
            armSubsystem.rotateArm((motorValue.getAsDouble() / 4));
            // System.out.println("Going back up into bounds");
        }

        else if(armSubsystem.getArmPosition() >= Constants.Arm.ampArmAngle && motorValue.getAsDouble() < -0.25){
            armSubsystem.rotateArm((motorValue.getAsDouble() / 4));
            // System.out.println("Going back down into bounds");

        }

        else{

            // new HoldArmCommand(armSubsystem.getArmPosition());
            armSubsystem.StopArm();
            // // System.out.println("stopping");
            // System.out.println(motorValue);
            // System.out.println(armSubsystem.getArmPosition());
        }

        //  armSubsystem.rotateArm((motorValue.getAsDouble() / 4));

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