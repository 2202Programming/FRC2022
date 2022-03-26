package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.DriverControls.Id;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.shooter.Shooter_Subsystem;


public class ChangeAdjFactors extends CommandBase { 

    final Shooter_Subsystem shooter;
    final DriverControls dc;

    final double adjFactor;

    NetworkTable table;
    NetworkTable drivetrainTable;
    NetworkTableEntry ntShort;
    NetworkTableEntry ntMedium;
    NetworkTableEntry ntLong;
    public final String NT_Name = "Shooter"; 
    
    public ChangeAdjFactors(double adjFactor, DriverControls dc){
        this.shooter = RobotContainer.RC().shooter;
        this.dc = dc;
        
        this.adjFactor = adjFactor;

        table = NetworkTableInstance.getDefault().getTable(NT_Name);

        ntShort = table.getEntry("/Adjustment Factors/Short Adjustment Factor");
        ntMedium = table.getEntry("/Adjustment Factors/Medium Adjustment Factor");
        ntLong = table.getEntry("/Adjustment Factors/Long Adjustment Factor");
    }


    @Override
    public void initialize(){
        if (dc.bind(Id.Driver, XboxButton.START).get()) {
            shooter.changeShortAdj(adjFactor);
            ntShort.setDouble(shooter.getShortAdj());
        } 
        if (dc.bind(Id.Driver, XboxButton.BACK).get()) {
            shooter.changeMediumAdj(adjFactor);
            ntMedium.setDouble(shooter.getMediumAdj());
        } 
        if (dc.bind(Id.Driver, XboxButton.A).get()) {
            shooter.changeLongAdj(adjFactor);
            ntLong.setDouble(shooter.getLongAdj());
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }


}
