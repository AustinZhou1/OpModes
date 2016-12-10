package org.firstinspires.ftc.teamcode;

/*
 * Program name:        NudgeControl
 * Author:              Sam Wagner
 * Original date:       11/23/2016
 * Modification date:   12/7/2016 by Coach Ollie
 *                      - Fixed confusion in brackets
 *                      - Fixed some logic errors
 *                      - Made some logic more readable
 *                      - Added abort transition for active nudge state
 *                      12/9/2016 by Coach Ollie
 *                      - Fixed gamepad inputs and one motor direction
 * Project description: This code is a TeleOp which allows the robot's
 *                      driver to use the nudge buttons on both controllers to steer the robot.
 *
 *                      This is the first TeleOp created for Metal Mario to be used in Hat Tricks
 *                      on Dec 10, 2016
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="NudgeControl", group="HatTricks")
public class NudgeControl extends LinearOpMode {
    private enum NudgeState {INIT, NORMAL,START, WAIT, EXTEND}
    private final double ZERO_ZONE      = 0.05; // in range 0.0 .. 1.0

    private DcMotor mtrRF, mtrRM, mtrRB;        // Right side motors
    private DcMotor mtrLF, mtrLM, mtrLB;        // Left side motors

                                                // Direction pad variables
    private boolean dpad_up, dpad_right, dpad_down, dpad_left;
    private boolean prev_up, prev_right, prev_down, prev_left;

                                                // Finite State Machine Variables
    private boolean nudgeClick;
    private NudgeState nudgeState = NudgeState.INIT;
    private long nudgeStarted, nudgeDuration;

                                                // Integrated gamepad values
    private double leftY, rightX;

                                                // Final motor control values
    private double robotSpeed, leftMultiplier, rightMultiplier;

    private void initHardware() {
                                                // Map the RC phone configuration for motors
        mtrRF = hardwareMap.dcMotor.get("mtrRF");
        mtrRM = hardwareMap.dcMotor.get("mtrRM");
        mtrRB = hardwareMap.dcMotor.get("mtrRB");
        mtrLF = hardwareMap.dcMotor.get("mtrLF");
        mtrLM = hardwareMap.dcMotor.get("mtrLM");
        mtrLB = hardwareMap.dcMotor.get("mtrLB");

                                                // Set directions
        mtrRF.setDirection(DcMotor.Direction.FORWARD);
        mtrRM.setDirection(DcMotor.Direction.FORWARD);
        mtrRB.setDirection(DcMotor.Direction.FORWARD);
        mtrLF.setDirection(DcMotor.Direction.REVERSE);
        mtrLM.setDirection(DcMotor.Direction.REVERSE);
        mtrLB.setDirection(DcMotor.Direction.REVERSE);

                                                // Run the motors with Speed Set Points
                                                // instead of PWM Power Outputs in MCM
        mtrRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void updateHardware() {
                                                // The multiplier range = -1.0 .. +1.0
                                                // The speed range      = -100.0 .. +100.0
        double leftSpeed    = leftMultiplier * robotSpeed;
        double rightSpeed   = rightMultiplier * robotSpeed;
                                                // No value clamping is required
        mtrRF.setPower(rightSpeed);
        mtrRM.setPower(rightSpeed);
        mtrRB.setPower(rightSpeed);
        mtrLF.setPower(leftSpeed);
        mtrLM.setPower(leftSpeed);
        mtrLB.setPower(leftSpeed);
    }

    private void mergeGamePads() {
                                                // gamepad1 has the priority
        if (gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_left) {
            dpad_up     = gamepad1.dpad_up;
            dpad_right  = gamepad1.dpad_right;
            dpad_down   = gamepad1.dpad_down;
            dpad_left   = gamepad1.dpad_left;
        } else {
            dpad_up     = gamepad2.dpad_up;
            dpad_right  = gamepad2.dpad_right;
            dpad_down   = gamepad2.dpad_down;
            dpad_left   = gamepad2.dpad_left;
        }
                                                // Ignore joysticks, if either dpad is used
        if (dpad_up || dpad_right || dpad_down || dpad_left) {
            leftY = 0.0;
            rightX = 0.0;
        } else {
            double y1   = gamepad1.left_stick_y;
            double x1   = gamepad1.right_stick_x;
            double y2   = gamepad2.left_stick_y;
            double x2   = gamepad2.right_stick_x;
            double sum1 = Math.abs(y1) + Math.abs(x1);
            double sum2 = Math.abs(y2) + Math.abs(x2);
                                                // Use the more active joystick
            if (sum1 > sum2) {
                leftY   = y1;
                rightX  = x1;
            } else {
                leftY   = y2;
                rightX  = x2;
            }
        }
    }

    private void smoothControl() {
                                                // Reverse the left stick y
                                                // Change the scale into percentage
                                                // Use x^3 function for smooth control
        robotSpeed      = -100.0 * leftY * leftY * leftY;

                                                // Default is that there are no turns
        rightMultiplier     = 1.0;
        leftMultiplier      = 1.0;
                                                // Use x^3 function for smooth control
        if (rightX >  ZERO_ZONE) rightMultiplier    = 1 - 2 * rightX * rightX * rightX;
        if (rightX < -ZERO_ZONE) leftMultiplier     = 1 + 2 * rightX * rightX * rightX;
    }

    private void detectNudgeClick() {
                                                // Detect any edge
        nudgeClick 	=   (dpad_up && !prev_up)       || (dpad_right && !prev_right)
                    ||  (dpad_down && !prev_down)   || (dpad_left && !prev_left);
                                                // Trace the previous values
        prev_up	    = dpad_up;
        prev_right	= dpad_right;
        prev_down	= dpad_down;
        prev_left	= dpad_left;
    }

    private void executeNudgeControl() {
        final long   NUDGE_DURATION = 50;   // in milliseconds
        final double NUDGE_SPEED    = 40.0; // in percents

        switch (nudgeState) {
            case INIT:                          // Be prepared to detect the nudge clicks
                prev_up = false;
                prev_right = false;
                prev_down = false;
                prev_left = false;
                nudgeState = NudgeState.NORMAL;
                break;

            case NORMAL:
                smoothControl();                // The normal drive controls
                detectNudgeClick();
                if (nudgeClick) {
                    nudgeState = NudgeState.START;
                }
                break;

            case START:
                robotSpeed = NUDGE_SPEED;       // Set predefined speed

                if (prev_up) {                   // Direction from 10:30 to 1:30 o'clock
                    // Default is 12:00 and no turns
                    rightMultiplier = 1.0;
                    leftMultiplier = 1.0;
                    // Stop right motors for direction 1:30
                    if (prev_right) rightMultiplier = 0.0;
                    // Stop left motors for direction 10:30
                    if (prev_left) leftMultiplier = 0.0;

                } else if (prev_down) {         // Direction is from 4:30 to 7:30 o'clock
                    // Default is 6:00 and no turns
                    // Drive in reverse rirection
                    rightMultiplier = -1.0;
                    leftMultiplier = -1.0;
                    // Stop right motors for direction 4:30
                    if (prev_right) rightMultiplier = 0.0;
                    // Stop left motors for direction 7:30
                    if (prev_left) leftMultiplier = 0.0;

                } else if (prev_right) {        // Direction is 3:00 => CW turn
                    rightMultiplier = 1.0;
                    leftMultiplier = -1.0;

                } else if (prev_left) {         // Direction is 9:00 => CCW turn
                    rightMultiplier = -1.0;
                    leftMultiplier = 1.0;

                } else {                        // No direction selected
                    rightMultiplier = 0.0;
                    leftMultiplier = 0.0;
                }

                                                // Start the nudge timer
                nudgeStarted = System.currentTimeMillis();
                nudgeDuration = NUDGE_DURATION;

                nudgeState = NudgeState.WAIT;
                break;

            case WAIT:
                boolean nudgeDone = (System.currentTimeMillis() - nudgeStarted) > nudgeDuration;
                boolean abortNudge = Math.abs(leftY) > ZERO_ZONE;

                detectNudgeClick();
                if (nudgeClick) {               // Additional clicks will extend the nudge time
                    nudgeState = NudgeState.EXTEND;
                } else if (nudgeDone) {         // Once nudge time expires, continue with
                                                // normal control with joysticks
                    nudgeState = NudgeState.NORMAL;
                                                // Abort nudge control by moving left y stick
                } else if (abortNudge) {
                    nudgeState = NudgeState.NORMAL;
                }
                break;

            case EXTEND:
                nudgeDuration += NUDGE_DURATION;
                nudgeState = NudgeState.WAIT;
                break;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            mergeGamePads();
            executeNudgeControl();
            updateHardware();

            telemetry.addData("Sticks",  "leftY = %.3f, rightX = %.3f", leftY, rightX);
            telemetry.addData("Controls",  "rSpd = %.1f, lMlt = %.2f, rMlt = %.2f",
                                robotSpeed, leftMultiplier, rightMultiplier);
            telemetry.addData("Position", "left = %d, right = %d",
                                mtrLM.getCurrentPosition(), mtrRM.getCurrentPosition());
            telemetry.update();
        }
    }
}
