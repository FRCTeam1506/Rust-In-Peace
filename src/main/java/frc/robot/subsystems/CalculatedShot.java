// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class CalculatedShot {

    private Shooter shooter;
        
    public CalculatedShot(Shooter shooter) {
        this.shooter = shooter;
    }

    public static double rX; //Robot X
    public static double rY;

    public static double arcHeight; //Height from hub y line to top of arc
    public static double totalHeight; //Height from ground to y line
    


}
