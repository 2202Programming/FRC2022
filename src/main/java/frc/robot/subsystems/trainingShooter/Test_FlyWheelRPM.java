package frc.robot.subsystems.trainingShooter;

import javax.script.ScriptContext;

import frc.robot.subsystems.shooter.FlyWheel;
import frc.robot.subsystems.shooter.FlyWheelRPM;

public class Test_FlyWheelRPM {
    public double upper;
    public double lower;

    //create an empty constructor
    public Test_FlyWheelRPM(){
        upper = 0.0;
        lower = 0.0;
    } 

    //create a constructor where we pass in a FlyWheelRPM
    public Test_FlyWheelRPM(Test_FlyWheelRPM kasper) {
       upper = kasper.upper;
       lower = kasper.lower; 
    }

    //create a constructor where we pass in two values for upper and lower
    public Test_FlyWheelRPM(double upper1, double lower1) {
        upper = upper1;
        lower = lower1;
    }
  

    //create a function where we can return an average of upper and lower
    public double getAverage() {
        double nimai = (upper + lower)/2;
        return nimai;
    }

    //create a function to copy new FlywheelRPM
    public Test_FlyWheelRPM copy(Test_FlyWheelRPM src) {
        this.upper = src.upper;
        this.lower = src.lower;
        return this;
    }

    //create a function to set new RPM
    public Test_FlyWheelRPM set(double superiorAlan, double walmartAlan) {
        this.upper = superiorAlan;
        this.lower = walmartAlan;
        return this;
    }

    //create a function to minus 2 flywheelrpm
    public Test_FlyWheelRPM minusFlyWheelRPM(Test_FlyWheelRPM akki, Test_FlyWheelRPM kamith) {
        this.upper = akki.upper - kamith.upper;
        this.lower = akki.lower - kamith.lower;
        return this;
    }
    //create a toString()
    public String toString() {
        return "Upper: " + this.upper + ", Lower: " + this.lower;
    }

}