package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware.Shooter;

@TeleOp
public class Test extends LinearOpMode {

    private CRServo ServoTurn;
    private DcMotorEx Intake;
    public Shooter shooter;

    // PID constants (tune these!)
    private double kP = 0.005;
    private double kI = 0.0;
    private double kD = 0.001;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;


    private double targetPosition = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private long lastTime = 0;

    private double stuck = 0;
    private static final double ERROR_TOLERANCE = 20;   // error range for considering "same"
    private static final long STUCK_TIME_MS = 500;      // time threshold for stuck detection
    private double lastStableError = 0;
    private long stableStartTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        ServoTurn = hardwareMap.get(CRServo.class, "ServoTurn");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");

        Shooter shooter = new Shooter(hardwareMap);


        Intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_bumper){
                Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Intake.setPower(-1);
            }else Intake.setPower(-0.4);
            // Set a target when button A pressed
            if (gamepad1.a) {
                targetPosition = 2600;
            }
            if (gamepad1.b){
                targetPosition = 5300;
            }
            if(gamepad1.dpad_up){
                targetPosition = 3900;
            }
            if (gamepad1.dpad_down){
                targetPosition = 1200;
            }
            if (gamepad1.dpad_left){
                targetPosition = -1500;
            }
            if (gamepad1.left_bumper){
                shooter.startShooter();
            }else shooter.stopShooter();
            if (gamepad1.x) {
                shooter.shoot();
                sleep(500);
                shooter.resetServo();
            }

            // Read encoder position
            double currentPos = Intake.getCurrentPosition();

            // --- PID control ---
            long currentTime = System.nanoTime();
            double deltaTime = (currentTime - lastTime) / 1e9; // seconds
            lastTime = currentTime;

            double error = targetPosition - currentPos;
            //integralSum += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;
            lastError = error;

            double output = (kP * error) + (kI * 0) + (kD * derivative);

            output = Math.max(-0.5, Math.min(0.5, output));

// === Stuck detection ===


            // === Apply servo control ===
            if (Math.abs(error) > 300) {

                ServoTurn.setPower(output);
//                if (Math.abs(error - lastStableError) <= ERROR_TOLERANCE) {
//                    // Error hasn't changed much
//                    if (stableStartTime == 0) stableStartTime = System.currentTimeMillis();
//
//                    // Check how long it's been constant
//                    if (System.currentTimeMillis() - stableStartTime >= STUCK_TIME_MS) {
//                        // Servo appears stuck
//                        if (output > 0) {
//                            stuck = targetPosition;
//                            targetPosition = currentPos - 500;
//                        } else if (output < 0) {
//                            stuck = targetPosition;
//                            targetPosition = currentPos + 500;
//                        }
//
//                        // Reset timer to avoid repeating too quickly
//                        stableStartTime = 0;
//                    }
//                }else {
//                    // Error is changing → not stuck
//                    stableStartTime = 0;
//                    lastStableError = error;
//                }
            } else {
                // reached target
                if (stuck != 0) {
                    targetPosition = stuck;
                    stuck = 0;
                }
                ServoTurn.setPower(0);
            }

            // Telemetry
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current Position", currentPos);
            telemetry.addData("Error", error);
            telemetry.addData("PID Output", output);
            telemetry.update();
        }
    }
}
