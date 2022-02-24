// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib2202.nt;

import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.annotation.ElementType;

/** Add your docs here. */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface NtOutput {
    String name() default "";
}
