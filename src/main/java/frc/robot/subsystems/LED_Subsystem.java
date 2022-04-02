package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LED_Subsystem extends SubsystemBase {
    CANifier _canifier = new CANifier(0);
    int _loopCount = 0;
    
    public LED_Subsystem() {
        
    }
    public void teleopPeriodic() {
        if(_loopCount++ > 10)
        {
            _loopCount = 0;
            System.out.println("Bus voltage is: " + _canifier.getBusVoltage());
        }
    }
}
