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

        double TO = 3.0;  //default timeout for testing

        this.addCommands(
             //goto Travers start
            //new MoveArmsTo(climber, "traverseStart", travers_start_ext , travers_start_rot, true, true).withTimeout(5),
            new MoveArmsTo(climber, "traverseStart", travers_start_ext, travers_start_rot, true, true).withTimeout(5),
            new MoveArmsTo(climber, "rotate-under", travers_start_ext, 45, true, true).withTimeout(TO),
            new MoveArmsTo(climber, "rotateunder2", travers_start_ext, 50, true, true).withTimeout(TO),
            new MoveArmsTo(climber, "rotate4grab2", travers_start_ext, 60, true, true).withTimeout(TO),
            new MoveArmsTo(climber, "extend-under",  24, 55, true, true).withTimeout(TO),
            new MoveArmsTo(climber, "rotateforgrab", 24, 40, true, true).withTimeout(5),
            new WaitCommand(10),
            new MoveArmsTo(climber, "partial-pullup2", partialpullupext, 0, true, true).withTimeout(TO),
            new WaitCommand(5),
            new MoveArmsTo(climber, "pullupMid", pullupext, midrot, true, true).withTimeout(TO),
            new WaitCommand(5),
            new MoveArmsTo(climber, "rotateCG1", pullupext, pulluprotForward, true, true).withTimeout(TO),
            new WaitCommand(5),
            new MoveArmsTo(climber, "extend to trav", travers_start_ext, pulluprotForward, true, true).withTimeout(TO),
            new WaitCommand(5),
            new MoveArmsTo(climber, "rotate to trav", travers_start_ext, travers_start_rot, true, true).withTimeout(TO)    
            
     );
    }

}
