// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** Add your docs here. */
public class PathSelector {
    public PathSelector(Joystick leJoystick, Joystick rJoystick){
        this.leJoystick = leJoystick;
        this.rJoystick = rJoystick;
    }
    Joystick leJoystick;
    Joystick rJoystick;
    public void GetButtonPresses(){
        if (leJoystick.getRawButton(1)){
            int lDirect = leJoystick.getPOV();
            int rDirect = rJoystick.getPOV();

            SmartDashboard.putNumber("leftstick pov", lDirect);
            SmartDashboard.putNumber("right pov", rDirect);
            if(rJoystick.getRawButtonPressed(1)){
                if (lDirect == 0){
                //to the coral
                    if (rDirect == 90){
                        //right side
                        SmartDashboard.putString("Pathselector", "Right Coral");
                    }
                    else if(rDirect==270){
                        //left side
                        SmartDashboard.putString("Pathselector", "Left Coral");
                    }
                }else if (lDirect == 90){
                    //to the right pole
                    //note:right trigger makes it go to the bottom of l/r sides

                    if (rDirect == 0){
                        //1st side aka top
                        SmartDashboard.putString("Pathselector", "Top Reef Right Pole");
                    }
                    else if (rDirect == 45){
                        //2nd side aka top right
                        SmartDashboard.putString("Pathselector", "Top Right Reef Right Pole");
                    }
                    else if (rDirect == 135){
                        //3rd side aka bottom right
                        SmartDashboard.putString("Pathselector", "Bottom Right Reef Right Pole");
                    }
                    else if (rDirect == 180){
                        //4th side aka bottom
                        SmartDashboard.putString("Pathselector", "Bottom Reef Right Pole");
                    }
                    else if (rDirect == 225){
                        //5th side aka bottom left
                        SmartDashboard.putString("Pathselector", "Bottom Left Reef Right Pole");
                    }
                    }else if (rDirect == 315){
                        //6th side aka top left
                        SmartDashboard.putString("Pathselector", "Top Left Reef Right Pole");
                    }
                else if (lDirect == 270){
                    //to the left pole
                    if (rDirect == 0){
                        //1st side aka top
                        SmartDashboard.putString("Pathselector", "Top Reef Left Pole");
                    }
                    else if (rDirect == 45){
                        //2nd side aka top right
                        SmartDashboard.putString("Pathselector", "Top Right Reef Left Pole");
                    }
                    else if (rDirect == 135){
                        //3rd side aka bottom right
                        SmartDashboard.putString("Pathselector", "Bottom Right Reef Left Pole");
                    }
                    else if (rDirect == 180){
                        //4th side aka bottom
                        SmartDashboard.putString("Pathselector", "Bottom Reef Left Pole");
                    }
                    else if (rDirect == 225){
                        //5th side aka bottom left
                        SmartDashboard.putString("Pathselector", "Bottom Left Reef Left Pole");
                    }
                    else if (rDirect == 315){
                        //6th side aka top left
                        SmartDashboard.putString("Pathselector", "Top Left Reef Left Pole");
                    }
                }
            }
        }
    }

}
