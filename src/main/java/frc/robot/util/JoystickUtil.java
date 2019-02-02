/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

public class JoystickUtil {
    /**
     * Class containing joystick values for the XBOX
     * 
     * @author Yair Ziv
     */
    public static final class XBOX {
        // Buttons
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LB = 5;
        public static final int RB = 6;
        public static final int BACK = 7;
        public static final int START = 8;
        public static final int STICK_LEFT = 9;
        public static final int STICK_RIGHT = 10;

        // POV
        public static final int POV_CENTER = -1;
        public static final int POV_UP = 0;
        public static final int POV_UP_RIGHT = 45;
        public static final int POV_RIGHT = 90;
        public static final int POV_DOWN_RIGHT = 135;
        public static final int POV_DOWN = 180;
        public static final int POV_DOWN_LEFT = 225;
        public static final int POV_LEFT = 270;
        public static final int POV_LEFT_UP = 315;
    }

    /**
     * Class containing joystick values for normal Joystick
     * 
     * @author Yair Ziv
     */
    public static final class JOYSTICK {
        // Buttons
        public static final int TRIGGER = 1;
        public static final int THUMB = 2;
    }
}
