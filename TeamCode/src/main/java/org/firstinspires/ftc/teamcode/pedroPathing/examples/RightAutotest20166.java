package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemIntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemRobotID;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemElevator;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemGrabber;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.startingPoseRight;
/*
import static org.firstinspires.ftc.teamcode.wayPoints.dropOffToSpecimenPickup;
import static org.firstinspires.ftc.teamcode.wayPoints.dropOffToSpecimenPickupHeading;
import static org.firstinspires.ftc.teamcode.wayPoints.specimenPickUpToSubmersible2;
import static org.firstinspires.ftc.teamcode.wayPoints.specimenPickUpToSubmersible2Heading;
import static org.firstinspires.ftc.teamcode.wayPoints.spike1ToDropOff;
import static org.firstinspires.ftc.teamcode.wayPoints.spike1ToDropOffHeading;
import static org.firstinspires.ftc.teamcode.wayPoints.startToSubmersible;
import static org.firstinspires.ftc.teamcode.wayPoints.startToSubmersibleHeading;
import static org.firstinspires.ftc.teamcode.wayPoints.submersibleToSpike1;
import static org.firstinspires.ftc.teamcode.wayPoints.submersibleToSpike1Heading;
//import static org.firstinspires.ftc.teamcode.wayPoints.*;
*/
/**
 * Testing autonomous using Pedro based on curved back and forth example
 *
 */
@Config
@Autonomous
@Disabled
public class RightAutotest20166 extends OpMode {
    private RobotConstants robotConstants = null;
    private Telemetry telemetryA;
    private SubSystemGrabber robotGrabber;
    private SubSystemElevator robotElevator;
    private SubSystemRobotID robotRobotID = null;


    public double botHeading = startingPoseRight.getHeading();
    private Follower follower;
    private SubSystemIntakeArm robotIntakeArm;
    private int robotID;
    private int pickupCount = 0;

    private enum AUTON_STATE
    {
        AUTON_START_STATE,
        LOWER_ELEVATOR_STATE,
        LOWER_ELEVATOR_SETUP_STATE,
        WAIT_AUTO_FINISHED,
        WAIT_PATH_DONE_STATE,
        PUSH_SAMPLES_STATE,
        //MOVE_TO_SAMPLE_FROM_OBSERVATION_ZONE_STATE,
        PICKUP_SPECIMEN_STATE,
        WAIT_TIMER_DONE_STATE,
        ELEVATOR_PICK_UP_SAMPLE,
        BACKUP_AND_LOWER,
        MOVE_TO_SPECIMEN_PICKUP,
        DO_NOTHING,
    }

    private AUTON_STATE currentAutonomousState = AUTON_STATE.AUTON_START_STATE;
    private AUTON_STATE waitPathDoneNextState = AUTON_STATE.AUTON_START_STATE;
    private AUTON_STATE lowerElevatorNextState = AUTON_STATE.AUTON_START_STATE;
    private AUTON_STATE waitTimerDoneNextState = AUTON_STATE.AUTON_START_STATE;

    private static ElapsedTime timeoutTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private int timeoutPeriod = 0;

    public static final Point startPoint = new Point (startingPoseRight.getX(), startingPoseRight.getY(), Point.CARTESIAN);
    public static final double wallY = startingPoseRight.getY() + 0.5;
    public static final double specimenWallPickupX = 35;//47.5;
    public static final double specimenWallPickupY = wallY;//-61.5?
    public static final double pushPrepY = -12.8;
    public static final double pushAlignY = -12.8;
    public static final double pushObservationY = -48;
    public static final Point submersibleDropPoint = new Point(2.4, -32, Point.CARTESIAN);

    //New Points for submersible to spikes ready to push into human player area
    public static final Point submersibleToSpike1 = new Point(29,-52 , Point.CARTESIAN);//First point to move away from sub
    public static final Point submersibleToSpike2 = new Point(33.4,-48.8 , Point.CARTESIAN);//Slide right ready to move behind samples
    public static final Point submersibleToSpike3 = new Point(33.4,pushPrepY, Point.CARTESIAN);//Drive forward so behind samples
    public static final Point submersibleToSpike4 = new Point(42,pushAlignY , Point.CARTESIAN);//Slide right until directly behind first sample
    public static final Point submersibleToSpike5 = new Point(42, pushObservationY , Point.CARTESIAN);//Observation drop 'push to' location
    public static final Point submersibleToSpike6 = new Point(42,pushPrepY , Point.CARTESIAN); //Back behind the samples
    public static final Point submersibleToSpike7 = new Point(54,pushAlignY , Point.CARTESIAN);//Aligned behind second sample
    public static final Point submersibleToSpike8 = new Point(54,pushObservationY , Point.CARTESIAN);//Observation drop 'push to' location for second sample
    public static final Point submersibleToSpike9 = new Point(54,pushPrepY , Point.CARTESIAN);//Back behind samples
    public static final Point submersibleToSpike10 = new Point(60,pushAlignY , Point.CARTESIAN);//Aligned behind third sample
    public static final Point submersibleToSpike11 = new Point(60,pushObservationY  , Point.CARTESIAN);//Observation drop 'push to' location for third sample
    public static final Point submersibleToSpike12 = new Point(52,-38  , Point.CARTESIAN);//Back out of Observation zone
    public static final Point submersibleToSpike13 = new Point(specimenWallPickupX,specimenWallPickupY  , Point.CARTESIAN);//Move into grabbing position
    //Paths
    public static final Path submersibleToSpikePathSegment1 = new Path(new BezierCurve(submersibleDropPoint, submersibleToSpike1, submersibleToSpike2));
    public static final Path submersibleToSpikePathSegment2 = new Path(new BezierCurve(submersibleToSpike2, submersibleToSpike3));
    public static final Path submersibleToSpikePathSegment3 = new Path(new BezierCurve(submersibleToSpike3, submersibleToSpike4));
    public static final Path submersibleToSpikePathSegment4 = new Path(new BezierCurve(submersibleToSpike4, submersibleToSpike5));
    public static final Path submersibleToSpikePathSegment5 = new Path(new BezierCurve(submersibleToSpike5, submersibleToSpike6));
    public static final Path submersibleToSpikePathSegment6 = new Path(new BezierCurve(submersibleToSpike6, submersibleToSpike7));
    public static final Path submersibleToSpikePathSegment7 = new Path(new BezierCurve(submersibleToSpike7, submersibleToSpike8));
    public static final Path submersibleToSpikePathSegment83x = new Path(new BezierCurve(submersibleToSpike8, submersibleToSpike9));
    public static final Path submersibleToSpikePathSegment82x = new Path(new BezierCurve(submersibleToSpike8, submersibleToSpike12));

    public static final Path submersibleToSpikePathSegment9 = new Path(new BezierCurve(submersibleToSpike9, submersibleToSpike10));
    public static final Path submersibleToSpikePathSegment10 = new Path(new BezierCurve(submersibleToSpike10, submersibleToSpike11));
    public static final Path submersibleToSpikePathSegment11 = new Path(new BezierCurve(submersibleToSpike11, submersibleToSpike12));
    public static final Path submersibleToSpikePathSegment12 = new Path(new BezierCurve(submersibleToSpike12, submersibleToSpike13));

     public static final PathChain submersibleToSpikeOneChain3x = new PathChain(submersibleToSpikePathSegment1, submersibleToSpikePathSegment2, submersibleToSpikePathSegment3, submersibleToSpikePathSegment4, submersibleToSpikePathSegment5, submersibleToSpikePathSegment6, submersibleToSpikePathSegment7, submersibleToSpikePathSegment83x, submersibleToSpikePathSegment9,submersibleToSpikePathSegment10, submersibleToSpikePathSegment11, submersibleToSpikePathSegment12);
    public static final PathChain submersibleToSpikeOneChain2x = new PathChain(submersibleToSpikePathSegment1, submersibleToSpikePathSegment2, submersibleToSpikePathSegment3, submersibleToSpikePathSegment4, submersibleToSpikePathSegment5, submersibleToSpikePathSegment6, submersibleToSpikePathSegment7, submersibleToSpikePathSegment82x, submersibleToSpikePathSegment12);
    public static  final Path startToSubmersible = new Path(new BezierCurve(startPoint, submersibleDropPoint));
    public static final double startToSubmersibleHeading = 90;
    public static final double submersibleToSpike1Heading = 90;

    //
    public static Point specimenPickupPoint = new Point(specimenWallPickupX,specimenWallPickupY  , Point.CARTESIAN);
    public static Point specimenTwoClipOn = new Point(6, -32.5, Point.CARTESIAN);
    public static Point specimenThreeClipOn = new Point(-2, -32.5, Point.CARTESIAN);
    public static final Path specimenTwoToSubmersible = new Path(new BezierCurve(specimenPickupPoint, specimenTwoClipOn));
    public static final Path moveToSpeciminThreePickup = new Path (new BezierCurve(specimenTwoClipOn, specimenPickupPoint));
    public static final Path specimenThreeToSubmersible = new Path(new BezierCurve(specimenPickupPoint, specimenThreeClipOn));
    public static Point autonHoldLocation = specimenPickupPoint;
    public static final Path moveToAutonHoldPath = new Path(new BezierCurve(specimenThreeClipOn, autonHoldLocation));

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void start() {
        restartTimeout(robotConstants.RIGHT_AUTON_DELAY);
    }
    @Override
    public void init()
    {
        try {
            robotRobotID = new SubSystemRobotID(hardwareMap);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robotID = robotRobotID.getRobotID();
        robotConstants = new RobotConstants(robotID);

        follower = new Follower(hardwareMap, robotConstants.DRIVE_DIRECTION);
        follower.setStartingPose(startingPoseRight);


        try {
            initializeSubSystems();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        follower.setMaxPower(robotConstants.START_TO_SUBMERSIBLE_SPEED);

        initalizePathHeadings();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

    }
    private void initalizePathHeadings()
    {
        submersibleToSpikePathSegment1.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment2.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment3.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment4.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment5.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment6.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment7.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment83x.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment82x.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment9.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment10.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment11.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment12.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(270), 0.7);
        moveToSpeciminThreePickup.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(270), 0.5);
    }
    private void initializeSubSystems() throws InterruptedException {
        robotElevator = new SubSystemElevator(hardwareMap, robotConstants.ELEVATOR_MULTIPLIER, robotConstants.ELEVATOR_MOTOR_COUNT);
        robotGrabber = new SubSystemGrabber(hardwareMap, 1);
        robotIntakeArm = new SubSystemIntakeArm(hardwareMap);
    }


    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */

    private void setupPath(Path pathToFollow, double endHeading)
    {
        double currentHeading = botHeading;
        follower.followPath(pathToFollow);
        pathToFollow.setLinearHeadingInterpolation(currentHeading, Math.toRadians(endHeading), 0.7);
        botHeading = endHeading;
    }
    private void setupPathChain(PathChain pathChainToFollow, double endHeading)
    {
        //MAKE SURE DEGREES OR RADIANS CORRECT
        double currentHeading = botHeading;
        follower.followPath(pathChainToFollow);
        botHeading = endHeading;
 //       initalizePathHeadings();
 //       int curveCount = pathChainToFollow.size();
 //       for (int i = 0; i < curveCount; i++)
 //       {
 //           pathChainToFollow.getPath(i).setLinearHeadingInterpolation(currentHeading, endHeading);
 //       }
    }

    private void restartTimeout(int timeout)
    {
        timeoutPeriod = timeout;
        timeoutTimer.reset();
    }

    private boolean pathIsBusy()
    {
        //if (hasTimededout())
          //  return false;
        //else if (follower.isBusy())
          //  return true;
        //else
          //  return false;
        if (robotStalled)
            return false;
        else
            return follower.isBusy();
    }

    private boolean hasTimededout()
    {
        if (timeoutTimer.time() < timeoutPeriod)
            return false;
        else
            return true;
    }

    private void processStateStart()
    {
        if (hasTimededout())
        {
            follower.setMaxPower(robotConstants.START_TO_SUBMERSIBLE_SPEED);
            setupPath(startToSubmersible, startToSubmersibleHeading);
            restartTimeout(8000);
            robotElevator.setPosition(robotConstants.ELEVATOR_TOP_RUNG_PLACE);
            currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
            waitPathDoneNextState = AUTON_STATE.LOWER_ELEVATOR_SETUP_STATE;
            lowerElevatorNextState = AUTON_STATE.PUSH_SAMPLES_STATE;
        }
    }

    private void processLowerElevatorSetupState()
    {
        robotElevator.setPosition(robotConstants.ELEVATOR_TOP_RUNG_RELEASE);
        restartTimeout(500);
        currentAutonomousState = AUTON_STATE.LOWER_ELEVATOR_STATE;
    }
    private void processLowerElevatorState()
    {
//        if((Math.abs(robotElevator.getPosition() - robotConstants.ELEVATOR_TOP_RUNG_RELEASE) < 5) || hasTimededout())
        if(robotElevator.atTargetYet(10))
        {
            robotGrabber.setPosition(robotConstants.GRABBER_OPEN_POSITION);
            currentAutonomousState = lowerElevatorNextState;
            robotElevator.setPosition(robotConstants.ELEVATOR_SPECIMEN_PICKUP);
        }
    }

    private void processWaitPathDone()
    {
        if (!pathIsBusy())
            currentAutonomousState = waitPathDoneNextState;
    }
    private void processWaitAutonDoneState()
    {
        follower.holdPoint(new BezierPoint(autonHoldLocation), Math.toRadians(90));
    }

    private void processPushSampleToObservation()
    {
        //setupPathChain(submersibleToSpikeOneChain3x, submersibleToSpike1Heading);
        setupPathChain(submersibleToSpikeOneChain2x, submersibleToSpike1Heading);
        follower.setMaxPower(robotConstants.SUBMERSIBLE_TO_PUSH_SPEED);
        restartTimeout(8000);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        waitPathDoneNextState = AUTON_STATE.PICKUP_SPECIMEN_STATE;
    }

    private void processMoveToSpecimenPickupState()
    {
        setupPath(moveToSpeciminThreePickup, 270);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        waitPathDoneNextState = AUTON_STATE.PICKUP_SPECIMEN_STATE;
    }
    private void processPickupSpecimenState()
    {
        pickupCount = pickupCount + 1;
        robotGrabber.setPosition(robotConstants.GRABBER_CLOSE_POSITION);
        restartTimeout(300);
        currentAutonomousState = AUTON_STATE.WAIT_TIMER_DONE_STATE;
        waitTimerDoneNextState = AUTON_STATE.ELEVATOR_PICK_UP_SAMPLE;
    }
    private void waitTimerDone()
    {
        if (hasTimededout())
        {
            currentAutonomousState = waitTimerDoneNextState;
        }
    }

    private void processElevatorPickUpSample()
    {
        robotElevator.setPosition(robotConstants.ELEVATOR_SPECIMEN_PICK_UP_LIFT_POSITION);
        if (robotElevator.atTargetYet(10))
        {
           restartTimeout(8000);
           if (pickupCount == 1) {
               setupPath(specimenTwoToSubmersible, 90);
               lowerElevatorNextState = AUTON_STATE.MOVE_TO_SPECIMEN_PICKUP;
           }
           else {
               setupPath(specimenThreeToSubmersible, 90);
               lowerElevatorNextState = AUTON_STATE.BACKUP_AND_LOWER;
           }
           robotElevator.setPosition(robotConstants.ELEVATOR_TOP_RUNG_PLACE);
           currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
           waitPathDoneNextState = AUTON_STATE.LOWER_ELEVATOR_SETUP_STATE;
        }
    }
    private Point poseToPoint(Pose pose)
    {
        Point returnPoint = new Point(pose.getX(), pose.getY(), Point.CARTESIAN);
        return returnPoint;
    }

    private void setPathFromCurrentPositionToTargetPose(Pose targetPose)
    {
        Point targetPoint = poseToPoint(targetPose);
        double targetHeading = targetPose.getHeading();
        setupPath(new Path(new BezierCurve(poseToPoint(follower.getPose()), targetPoint)),targetHeading);
    }

    private void setPathFromCurrentPositionToSubmersible(Pose endPose)
    {
        Point endPoint = poseToPoint(endPose);
        setupPath(new Path(new BezierCurve(poseToPoint(follower.getPose()), specimenTwoClipOn)),90);
    }
    private void processBackupAndLowerState()
    {
        setupPath(moveToAutonHoldPath, 90);
        robotElevator.setPosition(0);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        waitPathDoneNextState = AUTON_STATE.WAIT_AUTO_FINISHED;
    }
    private void doNothing()
    {
        //follower.holdPoint(new BezierPoint(specimenTwoClipOn), Math.toRadians(90));
    }
    private void processStateMachine()
    {
        switch (currentAutonomousState)
        {
            case AUTON_START_STATE:
                processStateStart();
                break;
            case WAIT_PATH_DONE_STATE:
                processWaitPathDone();
                break;
            case WAIT_AUTO_FINISHED:
                processWaitAutonDoneState();
                break;
            case LOWER_ELEVATOR_SETUP_STATE:
                processLowerElevatorSetupState();
                break;
            case LOWER_ELEVATOR_STATE:
                processLowerElevatorState();
                break;
            case PUSH_SAMPLES_STATE:
                processPushSampleToObservation();
                break;
            case PICKUP_SPECIMEN_STATE:
                processPickupSpecimenState();
                break;
            case WAIT_TIMER_DONE_STATE:
                waitTimerDone();
                break;
            case ELEVATOR_PICK_UP_SAMPLE:
                processElevatorPickUpSample();
                break;
            case MOVE_TO_SPECIMEN_PICKUP:
                processMoveToSpecimenPickupState();
                break;
            case BACKUP_AND_LOWER:
                processBackupAndLowerState();
                break;
            case DO_NOTHING:
                doNothing();
                break;
        }

    }

    private double[] deltaTracking = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    private int stallIndex = 0;
    private double deltaTotal = 10;
    private double lastX = 0;
    private double lastHeading = 0;
    private double lastY = 0;
    private double minDelta = 1000;

    private boolean robotStalled = false;
    private void checkIfStalled()
    {
        double distanceMoved = 0;
        if (follower.isBusy()) {


            double currentX = follower.getPose().getX();
            double distanceMovedX = currentX - lastX;
            lastX = currentX;
            double currentY = follower.getPose().getY();
            double distanceMovedY = currentY - lastY;
            lastY = currentY;
            double currentHeading = follower.getPose().getHeading();
            double distanceMovedHeading = currentHeading - lastHeading;
            lastHeading = currentHeading;
            distanceMoved = Math.abs(distanceMovedX) + Math.abs(distanceMovedY) + Math.abs(distanceMovedHeading * 5);
            deltaTotal = deltaTotal - deltaTracking[stallIndex] + distanceMoved;
            if (deltaTotal < minDelta)
                minDelta = deltaTotal;
            deltaTracking[stallIndex] = distanceMoved;
            if (stallIndex < 9)
                stallIndex = stallIndex + 1;
            else
                stallIndex = 0;

            if (deltaTotal < robotConstants.STALL_THRESHOLD)
                robotStalled = true;
            else
                robotStalled = false;

        }
        /*telemetryA.addData("Distance Moved", distanceMoved);
        telemetryA.addData("deltaTotal", deltaTotal);
        telemetryA.addData("minDelta", minDelta);
        telemetryA.addData("robotStalled", robotStalled);*/
        //robotStalled = false;

    }


    public void loop() {
        processStateMachine();
        follower.update();
        checkIfStalled();

        telemetryA.addData("Timer", timeoutTimer.time());
        telemetryA.addData("Current state",currentAutonomousState);
        telemetryA.addData("Elevator target: ", robotElevator.getTarget());
        telemetryA.addData("Elevator current: ", robotElevator.getPosition());
        follower.telemetryDebug(telemetryA);
    }
}
