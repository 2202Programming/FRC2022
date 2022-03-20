// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.Climber;
//import static frc.robot.commands.climber.MidClimb.travers_start_ext;
import static frc.robot.commands.climber.MidClimb.*;


public class TraverseClimb extends SequentialCommandGroup {
    
    // dirve.toClimbStart
    // climbMid
    // stabilize etc.

    public TraverseClimb(Climber climber) {
        super();
        
        double hang_rot = 0;

        double TO = 3.0;  //default timeout for testing

        this.addCommands(
             //goto Travers start
            new MoveArmsTo(climber, "traverseStart", travers_start_ext , travers_start_rot, true, true).withTimeout(2),
            new MoveArmsTo(climber, "extforgrab", 20 , travers_start_rot, true, true).withTimeout(2),
            new MoveArmsTo(climber, "rotateforgrab", 20 , -30, true, true).withTimeout(2),
            new MoveArmsTo(climber, "pullupTraverse", pullupext, -30, true, true).withTimeout(TO),

            // new MoveArmsTo(climber, pullupext, -5, true, true).withName("rotateCG1").withTimeout(TO),
            // new MoveArmsTo(climber, pullupext, hang_rot, true, true).withName("rotateCG2").withTimeout(TO),
            // new MoveArmsTo(climber, midext, hang_rot, true, true).withName("extend to hang").withTimeout(TO),
            new WaitCommand(10).withName("hangingOnTraverse")
     );
    }

}
