// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

//import static frc.robot.commands.climber.MidClimb.travers_start_ext;
import static frc.robot.commands.climber.MidClimb.partialpullupext;
import static frc.robot.commands.climber.MidClimb.pullupext;
import static frc.robot.commands.climber.MidClimb.pulluprotForward;
import static frc.robot.commands.climber.MidClimb.travers_start_ext;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.Climber;

public class TraverseClimb extends SequentialCommandGroup {

    // dirve.toClimbStart
    // climbMid
    // stabilize etc.

    //private Climber climber;
    //private LinearFilter filter;

    public TraverseClimb(Climber climber) {
        super();

        double TO = 2.0; // default timeout for testing

        this.addCommands(
                // goto Travers start
                // new MoveArmsTo(climber, "traverseStart", travers_start_ext ,
                // travers_start_rot, true, true).withTimeout(5),
                // check how we are swinging before traversing - neg is robot swing to lower bars
                // static hang is about +1deg.
                // allow a swing towards the bar
                new SwingCheck(SwingCheck.Axis.Pitch, -20, 1.0, -4.0, 1.0), //min/max angle, min/max rate
            /// step removed 4/4/22 test    
            ///new MoveArmsTo(climber, "extend to trav", travers_start_ext, pulluprotForward, true, true)
            ///        .withTimeout(TO),
                new MoveArmsTo(climber, "rotateunder2", travers_start_ext, 53, true, true).withTimeout(TO),
                new MoveArmsTo(climber, "extend-under", 23, 53, true, true).withTimeout(TO),
                new MoveArmsTo(climber, "rotateforgrab  +35", 23, 35, true, true).withTimeout(2),
                new MoveArmsTo(climber, "partial-pullup 0", partialpullupext, 0, true, true).withTimeout(TO),
                //put magic counterswing cmd here...
                new MoveArmsTo(climber, "partial-pullup -18", partialpullupext, -22, true, true).withTimeout(TO),
                new MoveArmsTo(climber, "pullupMid2", pullupext, -17, true, true).withTimeout(TO),
                new MoveArmsTo(climber, "rotateCG1", pullupext, pulluprotForward, true, true).withTimeout(TO));
        //this.climber = climber;
        //filter = LinearFilter.movingAverage(3);
    }

}
