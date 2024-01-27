package frc.robot.commands.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;


public class ExtendArmCommand extends Command{

    ArmSubsystem armSubsystem;
    
    public ExtendArmCommand(ArmSubsystem armSubsystem){
        
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);

    }

    @Override
    public void initialize(){
        
        armSubsystem.extendArm();

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