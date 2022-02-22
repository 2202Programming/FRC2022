// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib2202.nt;

import java.lang.reflect.Field;
import java.util.LinkedList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NtBind extends SubsystemBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("root");
  int count;

  interface apply {
    Boolean apply();
  }

  class doubleApply implements apply {
    public doubleApply(NtBind ntBind, NetworkTable table, String name, Field f, Object state) {
      var nt = table.getEntry(name);

      this.current = () -> {
        try {
          return f.getDouble(state);
        } catch (Exception e) {
          ntBind.ReportError(table.getPath(), name, e);
          return 0.0;
        }
      };

      this.set = (Double val) -> {
        try {
          f.setDouble(state, val);
        } catch (Exception e) {
          ntBind.ReportError(table.getPath(), name, e);
        }
      };

      this.next = () -> nt.getDouble(this.current.get());
    }

    Supplier<Double> current;
    Consumer<Double> set;
    Supplier<Double> next;

    public Boolean apply() {
      var next = this.next.get();
      var current = this.current.get();

      if (next != current) {
        set.accept(next);
        return true;
      }

      return false;
    }
  }

  class boundData implements apply {
    LinkedList<apply> applys = new LinkedList<apply>();
    Runnable onChange;

    public Boolean apply() {
      Boolean isChanged = false;

      for (var a : applys) {
        isChanged |= a.apply();
      }

      if(isChanged && onChange != null) {
        onChange.run();
      }
 
      return isChanged;
    }
  }

  LinkedList<boundData> boundObjects = new LinkedList<boundData>();

  public <TStateObject> TStateObject bind(TStateObject state, String name, Runnable onChange) {
    var classFields = state.getClass().getFields();
    var data = new boundData();
    var sub = table.getSubTable(name);

    for (var field : classFields) {
      for (var annotation : field.getAnnotations()) {

        if (annotation instanceof NtInput) {
          var ntInput = (NtInput) annotation;

          // figure out we should call it
          var entryName = ntInput.name();
          if (entryName == "") {
            entryName = field.getName();
          }

          if (field.getType().equals(double.class)) {
            data.applys.add(new doubleApply(this, sub, entryName, field, state));
          }
        }
      }
    }

    boundObjects.add(data);
    return state;
  }

  public void ReportError(String table, String name, Exception e) {

  }

  /** Creates a new NtBind. */
  public NtBind() {
  }

  @Override
  public void periodic() {
    count++;
    if (count > 10) {
      count = 0;

      for (var data : boundObjects) {
        data.apply();
      }
    }
    // This method will be called once per scheduler run
  }
}
