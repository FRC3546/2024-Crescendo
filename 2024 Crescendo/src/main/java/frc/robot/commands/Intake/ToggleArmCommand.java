package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ToggleArmCommand extends Command{

    ArmSubsystem armSubsystem;
    
    public ToggleArmCommand(ArmSubsystem armSubsystem){
        
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);

    }

    @Override
    public void initialize(){
        
        armSubsystem.toggleArm();

    }

    @Override
    public void execute() {
        
        

    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

