//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@TeleOp
//public class MotorControl extends LinearOpMode {
//
//    public DcMotor Shooter;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Shooter = hardwareMap.get(DcMotor.class,"Shooter");
//
//        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//
//        waitForStart();
//
//        while (opModeIsActive()){
//            //double ticks = Motor.getCurrentPosition();
//            int a = Motor.getCurrentPosition();
//            if (gamepad1.a){
//                Motor.setTargetPosition(a + 100);
//                Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                Motor.setPower(0.2);
//            }
//            if (gamepad1.b){
//                Motor.setTargetPosition(a - 100);
//                Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                Motor.setPower(0.2);
//            }
//            if (gamepad1.y){
//                Motor.setTargetPosition(0);
//                Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                Motor.setPower(0.2);
//            }
//
//            telemetry.addData("position",Motor.getCurrentPosition());
//            telemetry.update();
//
//
//
//
//
//        }
//
//
//
//    }
//}
