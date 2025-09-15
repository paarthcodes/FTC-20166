package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.robot.Robot;

@Config
public class RobotConstants
{

    public static final double STALL_X_DISREGARD_THRESHOLD = 10;
    public static final double STALL_Y_DISREGARD_THRESHOLD = 10;
    public static final double STALL_T_DISREGARD_THRESHOLD = 10;


    public static final double OBSERVATION_TO_SUBMERSIBLE_SPEED = 1;
    public static double ELEVATOR_MULTIPLIER;
    public static int DRIVE_DIRECTION;
    public static int ELEVATOR_MOTOR_COUNT;
    public static final int ELEVATOR_BOTTOM_POSITION = 0;//Lowest position, on the floor
    public static final int ELEVATOR_TOP_RUNG_PLACE = -2000;//Upper rung starting position

    public static final int ELEVATOR_TOP_RUNG_RELEASE = -1700;//Upper rung pull down position
    public static final int ELEVATOR_TOP_BASKET = -3670;//Top basket location
    public static final int ELEVATOR_LOW_BASKET = -2550;//Low basket location
    public static final int ELEVATOR_TRANSFER_SAMPLE_POSITION = -253;//sample transfer height
    public static final int LEFT_AUTON_DELAY = 0; //Delay in ms
    public static final int RIGHT_AUTON_DELAY = 0; //Delay in ms


    //   private static final int ELEVATOR_BOTTOM_RUNG_PLACE = -400;

    //   private static final int ELEVATOR_BOTTOM_RUNG_RELEASE = -225;

    //   private static final int ELEVATOR_TOP_BASKET = -1401;

    //   private static final int ELEVATOR_MIDDLE_BASKET = -896;

    public static final int ELEVATOR_SPECIMEN_PICK_UP_LIFT_POSITION = -1100;
    public static final int ELEVATOR_SPECIMEN_PICKUP = -730;//Was -830
    public static final double GRABBER_OPEN_POSITION = 0.7;
    public static final double GRABBER_CLOSE_POSITION = 0.95;
    public static double INTAKE_ARM_TRANSFER_SAMPLE_POSITION = 0.23;
    public static double INTAKE_ARM_UP_POSITION = 0;
    public static double INTAKE_ARM_DOWN_POSITION = 0.72;

    public static double START_TO_SUBMERSIBLE_SPEED;
    public static  double SUBMERSIBLE_TO_PUSH_SPEED;
    public static double BASKET_TO_SAMPLE_SPEED;

    public static  double INTAKE_ROLLER_OUT_SPEED = 1;
    public static  double INTAKE_ROLLER_IN_SPEED = -1;
    public static  double INTAKE_ROLLER_HOLD_SPEED = -0.2;
    public static double STALL_THRESHOLD;
    public static double INTAKE_PIVOT_PICKUP_POSITION = 0.4;
    public static double INTAKE_PIVOT_IDLE_POSITION = 0.5;
    public static final int STALL_SAMPLE_COUNT = 10;
    public static double INTAKE_SLIDE_MAX_POSITION = 0.78;
    public static double INTAKE_SLIDE_MIN_POSITION = 0;
    public static double INTAKE_SLIDE_SAFE_POSITION = 0.4;
    public static double INTAKE_SLIDE_START_POSITION = 0.0;
    public static double INTAKE_ARM_START_POSITION = 0.0;
    public static double INTAKE_PIVOT_START_POSITION = 0.0;

    public static int CLIMBER_UP_POSITION = 3400; //
    public static int CLIMBER_HANG_POSITION = 1000;

    public static int CLIMBER_SPEED = 100;


    public RobotConstants(int robotId)
    {
        if (robotId == 0)
        { //Bot A (V1)
            STALL_THRESHOLD = 0.02;
            ELEVATOR_MULTIPLIER = 1;
            START_TO_SUBMERSIBLE_SPEED = 0.8;
            BASKET_TO_SAMPLE_SPEED = 0.7;
            SUBMERSIBLE_TO_PUSH_SPEED = 0.9;
            DRIVE_DIRECTION = 1;
            ELEVATOR_MOTOR_COUNT = 1;
        } else if (robotId == 1)
        { //Bot C (V2)
            STALL_THRESHOLD = 0.02;
            ELEVATOR_MULTIPLIER = -0.96;
            START_TO_SUBMERSIBLE_SPEED = 0.73;
            BASKET_TO_SAMPLE_SPEED = 0.7;
            SUBMERSIBLE_TO_PUSH_SPEED = 1;
            DRIVE_DIRECTION = 1;
            ELEVATOR_MOTOR_COUNT = 1;
        } else
        { //Bot B (V3)
            STALL_THRESHOLD = 0.15;
            ELEVATOR_MULTIPLIER = 0.96;
            START_TO_SUBMERSIBLE_SPEED = 0.6;
            BASKET_TO_SAMPLE_SPEED = 0.7;
            SUBMERSIBLE_TO_PUSH_SPEED = 0.7;
            DRIVE_DIRECTION = -1;
            ELEVATOR_MOTOR_COUNT = 2;
        }
    }
}
