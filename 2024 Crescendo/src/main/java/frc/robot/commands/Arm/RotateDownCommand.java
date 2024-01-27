package frc.robot.commands.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class RotateDownCommand extends Command{

ArmSubsystem armSubsystem;

 public RotateDownCommand(ArmSubsystem armSubsystem){
        
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);

    }

    @Override
    public void initialize(){
        
        armSubsystem.rotateArmDown();

    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}