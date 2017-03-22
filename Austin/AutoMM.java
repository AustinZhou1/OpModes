package org.firstinspires.ftc.teamcode;

/*
 *  Program: AutoModeMM
 *  Author: Austin Zhou
 *  Date: 12-7-2016
 *  Modified:
 *  Description:  Autonomous mode for FTC robot.  Meant for moving robot in challenges of 2016-17.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.lang.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamlibrary.*;

/*
@Autonomous(name="AutoMM", group="Testing")
public class AutoMM extends LinearOpMode {

    static final double FORWARD_SPEED = 0.5;

    long DRIVEDURATION = 1700;
    private long driveStarted;
    private double robotSpeed, filteredSpeed, leftMultiplier, rightMultiplier, speedFF;
    private DataLogger              dl;
    private TCS34725_ColorSensor    cs;
    private long                    phaseStart;
    private int                     gain    = 1;
    private int                     iTime   = 4;
    int     gainValue;
    double  timeValue;



        public void init() {
            dl  = new DataLogger("CS_Test");
            cs  = new TCS34725_ColorSensor(hardwareMap,"color_sensor");
            dl.addField("Gain");
            dl.addField("ITime");
            dl.addField("Clr");
            dl.addField("Red");
            dl.addField("Blue");
            dl.addField("Green");
            dl.addField("CCT");
            dl.newLine();
        }

        public void init_loop() {
            telemetry.addData("isIdOk", cs.isIdOk());
            telemetry.addData("CCT", cs.colorTemp());
        }

        public void start() {
            phaseStart  = System.currentTimeMillis();
            cs.setGain(gain);
            cs.setIntegrationTime(iTime);
            gainValue   = cs.getGain();
            timeValue   = cs.getIntegrationTime();
        }


    @Override

        public void loop() {
            dl.addField(gainValue);
            dl.addField(timeValue);
            dl.addField(cs.clearColor());
            dl.addField(cs.redColor());
            dl.addField(cs.blueColor());
            dl.addField(cs.greenrColor());
            dl.addField(cs.colorTemp());
            dl.newLine();
            if (System.currentTimeMillis() - phaseStart >= 1000) {
                phaseStart  = System.currentTimeMillis();
                telemetry.addData("gain", gain);
                telemetry.addData("iTime", iTime);
                telemetry.addData("Clear", cs.clearColor());
                telemetry.addData("CCT", cs.colorTemp());

                gain    *= 4;
                if (gain>64) {
                    gain    = 1;
                    iTime  *= 4;
                    if (iTime > 1024) {
                        iTime   = 4;
                    }
                    cs.setIntegrationTime(iTime);
                    timeValue   = cs.getIntegrationTime();
                }
                cs.setGain(gain);
                gainValue   = cs.getGain();
            }
        }

        public void stop() {
            dl.closeDataLogger();
            cs.close();
        }

    public void runOpMode() throws InterruptedException {
        DcMotor mtrRF = hardwareMap.dcMotor.get("mtrRF");
        DcMotor mtrRM = hardwareMap.dcMotor.get("mtrRM");
        DcMotor mtrRB = hardwareMap.dcMotor.get("mtrRB");
        DcMotor mtrLF = hardwareMap.dcMotor.get("mtrLF");
        DcMotor mtrLM = hardwareMap.dcMotor.get("mtrLM");
        DcMotor mtrLB = hardwareMap.dcMotor.get("mtrLB");

        mtrRF.setDirection(DcMotor.Direction.FORWARD);
        mtrRM.setDirection(DcMotor.Direction.FORWARD);
        mtrRB.setDirection(DcMotor.Direction.FORWARD);
        mtrLF.setDirection(DcMotor.Direction.REVERSE);
        mtrLM.setDirection(DcMotor.Direction.REVERSE);
        mtrLB.setDirection(DcMotor.Direction.REVERSE);

        mtrRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        waitForStart();

        filteredSpeed = robotSpeed * speedFF + filteredSpeed * (1.0 - speedFF);
        driveStarted = (System.currentTimeMillis());


        mtrLB.setPower(filteredSpeed * 100);
        mtrLM.setPower(filteredSpeed * 100);
        mtrLF.setPower(filteredSpeed * 100);
        mtrRB.setPower(filteredSpeed * 100);
        mtrRM.setPower(filteredSpeed * 100);
        mtrRF.setPower(filteredSpeed * 100);

        while(opModeIsActive() &&
            ((System.currentTimeMillis()- driveStarted) < DRIVEDURATION)) {
            telemetry.addData("Position", "left = %d, right = %d", mtrLM.getCurrentPosition(), mtrRM.getCurrentPosition());
            telemetry.update();
        }

        mtrLB.setPower(0);
        mtrLM.setPower(0);
        mtrLF.setPower(0);
        mtrRB.setPower(0);
        mtrRM.setPower(0);
        mtrRF.setPower(0);
    }


}
*/
