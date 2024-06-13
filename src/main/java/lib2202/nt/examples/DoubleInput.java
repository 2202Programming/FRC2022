package lib2202.nt.examples;


import lib2202.nt.NtInput;
import lib2202.nt.NtOutput;
import lib2202.nt.NtBind;

public class DoubleInput {
    class State {

        @NtInput(name = "set")
        public double SetPoint = 0;

        @NtOutput(name = "err")
        public double Error = 0;
        
        // @NtInput()
    }

    State left = NtBind.getInstance().bind(new State(), "left", this::onChanged);
    State right = NtBind.getInstance().bind(new State(), "right", this::onChanged);

    void onChanged() {
        System.out.println("SetPoint: " + left.SetPoint + ", err: " + left.Error);
        System.out.println("SetPoint: " + right.SetPoint + ", err: " + right.Error);
    }
    
}
