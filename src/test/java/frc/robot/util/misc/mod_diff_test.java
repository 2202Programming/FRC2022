package frc.robot.util.misc;

import java.util.function.DoubleConsumer;

import org.junit.Before;
import org.junit.Test;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.ModMath;

public class mod_diff_test {
   static double tol = 1e-8;
   static double MOD360 = 360.0;

   public abstract interface DoubleFunc {
     public abstract double getAsDouble(double x);
  }

  @Before
  public void setUp() {
  }

  @Test
  public void test_mod_diff() {

      //  a1 - a0 on +-180 CCW+
       assert ModMath.delta_mod(179, -179, MOD360) == -2;
       assert ModMath.delta_mod(-179, 179, MOD360) == +2;
       assert ModMath.delta_mod(1, -1, MOD360) == +2;
       assert ModMath.delta_mod(-1, 1, MOD360) == -2;
       assert ModMath.delta_mod(0, -90, MOD360) == +90;
       assert ModMath.delta_mod(0, 90, MOD360) == -90;
       assert ModMath.delta_mod(90, -90, MOD360) ==  +180.0;
       assert ModMath.delta_mod(-90, 90, MOD360) == -180;

       // test the 360 variant
       assert ModMath.delta360(179, -179) == -2;
       assert ModMath.delta360(-179, 179) == +2;
       assert ModMath.delta360(1, -1) == +2;
       assert ModMath.delta360(-1, 1) == -2;
       assert ModMath.delta360(0, -90) == +90;
       assert ModMath.delta360(0, 90) == -90;
       assert ModMath.delta360(90, -90) ==  +180.0;
       assert ModMath.delta360(-90, 90) == -180;

      double pi2 = Math.PI*2;
      double sa = .01;
      double apos = pi2 - sa;
      double aneg = -apos;
      double sa2 = 2*sa;
     
      // check pi radians against small angle  around zero and pi2
      assert Math.abs(ModMath.delta_mod(apos, aneg, pi2) -  (-sa2))  < tol ;   
      assert Math.abs(ModMath.delta_mod(aneg, apos, pi2) -   sa2)  < tol ;
      assert Math.abs(ModMath.delta_mod(sa, -sa, pi2)    -   sa2)  < tol ;
      assert Math.abs(ModMath.delta_mod(-sa, sa, pi2)    - (-sa2))  < tol ;  
  }

  @Test
  public void test_fmod() {
    domainCheck(ModMath::fmod360);
  }

  @Test
  public void test_fmod_2() {
    domainCheck(ModMath::fmod360_2);
  }

  @Test
  public void test_equal() {
    for(double d = -179.9; d <= 180; d += 0.5) {     // test the domain
      for(int n = -20; n <= 20; n++) {               // tests the mod range
        double x = n*MOD360 + d;
        double val =  ModMath.fmod360(x) - ModMath.fmod360_2(x);
        if ( Math.abs(val) < tol) {
          assert true;
        } else {
          System.out.println("problem found d,n,x=" + d +", "+ n + ", " + x + " val = " +val);
          assert false;
        }
      }
    }    
  }

  void domainCheck (DoubleFunc f) {
    for(double d = -179.9; d <= 180; d += 0.5) {     // test the domain
      for(int n = -20; n <= 20; n++) {               // tests the mod range
        double x = n*MOD360 + d;
        double modval = f.getAsDouble(x);  
        assert Math.abs(modval - d) < tol;            // mod value should == domain
      }
    }
  }


  @Test
  public void test_delta() {
    
    for(double d = -179.9; d <= 180; d += 1.0) {     // test the domain
      for(int n = -10; n <= 10; n++) {               // tests the mod range    
        double neo_pos = d + n*MOD360;               // neo pos can extend beyond +/-180

        // now add some offset based on where we want to go, check the deltas make sense
        // 
        for (double o = -179.99; o <= 180.0; o += 5.0) {
          double delta = ModMath.delta360(o, neo_pos);
          double delta_op = ModMath.delta_mod(neo_pos, o, MOD360);

          assert (Math.abs( delta + delta_op ) < tol);

          // do simple delta, then take mod
          double delta_raw = o - neo_pos;
          double delta_bounded = ModMath.fmod360(delta_raw);
          
          double d_delta = delta - delta_bounded;

          if (Math.abs(d_delta) < tol) {
            assert true;
          }else {
            System.out.println("Problem " + "offset=" +o +" neo=" +neo_pos);
            assert false;
          }
        }
      }
    }     
  }




  @Test
  public void test_timing_fmod360() {
    testloop("fmod360", ModMath::fmod360);
  }

  @Test
  public void test_timing_fmod360_2() {
      testloop("fmod360_2", ModMath::fmod360_2);
  }

  static int LOOP = 10;
  static double DOMAIN = 65000.0;  //input range
  void testloop(String name, DoubleConsumer f ) {
    long startTime = RobotController.getFPGATime(); 
    for (int i = 0; i < LOOP; i++)
      for (double x = -DOMAIN; x < DOMAIN; x += +1.0) {
        f.accept(x);
      }
    long endTime = RobotController.getFPGATime();

    System.out.println("*** "+ name + " time = " + (endTime - startTime)/1000.0 + " mS");
  }

}
