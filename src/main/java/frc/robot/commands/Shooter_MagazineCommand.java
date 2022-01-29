// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.shooter.Shooter_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;

public class Shooter_MagazineCommand extends CommandBase {
 
    private Shooter_Subsystem shooter;
    public Magazine_Subsystem magazine;

enum Stage{
    DoNothing, 
    Loading, 
    WaitingForFlywheel, 
    Shooting, 
} Stage stage;

   public Shooter_MagazineCommand(Shooter_Subsystem shooter, Magazine_Subsystem magazine) {
    addRequirements(shooter);  
    this.shooter = shooter;
    this.magazine = magazine;
   }
    @Override 
    public void initialize() {
        stage = Stage.DoNothing;

       SmartDashboard.putNumber("Upper RPM", 0);
       SmartDashboard.putNumber("Lower RPM", 0);

       SmartDashboard.putNumber("Belt Speed", 0);
    }
    @Override
    public void execute() {
        switch(stage) {
            case DoNothing:
                double beltSpeed = SmartDashboard.getNumber("Belt Speed", 0);
                magazine.beltOn(beltSpeed);
                stage = Stage.Loading;
            break;

            case Loading:
                double upper = SmartDashboard.getNumber("Upper RPM", 0);
                double lower = SmartDashboard.getNumber("Lower RPM", 0);
                shooter.setMotors(upper, lower);
                stage = Stage.WaitingForFlywheel;
            break;

            case WaitingForFlywheel:
                if (shooter.isReadyToShoot()){
                    stage = Stage.Shooting;
                }
            break;

            case Shooting:
                if (!shooter.isReadyToShoot()) {
                    stage = Stage.WaitingForFlywheel;
                }
            break;
        }


    }
    @Override
    public boolean isFinished() {
        if (magazine.isGateBlocked()) {
            return false;
        } else {
            return true;
        }
    }
    @Override
    public void end(boolean interrupted) {
       shooter.setMotors(0, 0);
       magazine.beltOff();
    }
}
