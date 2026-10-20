
/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files.
 * The Software is provided "AS IS", without warranty of any kind.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

@TeleOp(name = "Yellow Ball Detector", group = "Vision")
public class ConceptVisionColorLocator_Cirlce extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Configure the yellow ball detector.
        ColorBlobLocatorProcessor colorLocator =
                new ColorBlobLocatorProcessor.Builder()
                        .setTargetColorRange(ColorRange.YELLOW)
                        .setContourMode(
                                ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                        .setRoi(ImageRegion.asUnityCenterCoordinates(
                                -0.85, 0.85, 0.85, -0.85))
                        .setDrawContours(true)
                        .setBoxFitColor(0)
                        .setCircleFitColor(Color.rgb(255, 255, 0))
                        .setBlurSize(5)

                        // Fill small gaps without excessive distortion.
                        .setDilateSize(5)
                        .setErodeSize(5)
                        .setMorphOperationType(
                                ColorBlobLocatorProcessor
                                        .MorphOperationType.CLOSING)
                        .build();

        // Connect to the webcam configured in the Robot Controller.
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(
                        WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        telemetry.addLine("Yellow Ball Detector Ready");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {

                // Retrieve detected yellow blobs.
                List<ColorBlobLocatorProcessor.Blob> blobs =
                        colorLocator.getBlobs();

                // Filter out small detections and irregular shapes.
                ColorBlobLocatorProcessor.Util.filterByCriteria(
                        ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                        300, 50000, blobs);

                ColorBlobLocatorProcessor.Util.filterByCriteria(
                        ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                        0.45, 1.0, blobs);

                telemetry.addData("Yellow blobs detected", blobs.size());

                if (!blobs.isEmpty()) {

                    // The largest contour is generally listed first.
                    ColorBlobLocatorProcessor.Blob bestBlob = blobs.get(0);
                    Circle circle = bestBlob.getCircle();

                    telemetry.addLine("YELLOW BALL FOUND");
                    telemetry.addData(
                            "Center X", (int) circle.getX());
                    telemetry.addData(
                            "Center Y", (int) circle.getY());
                    telemetry.addData(
                            "Radius", (int) circle.getRadius());
                    telemetry.addData(
                            "Circularity",
                            String.format("%.3f",
                                    bestBlob.getCircularity()));

                } else {
                    telemetry.addLine("No yellow ball detected");
                }

                telemetry.update();
                sleep(50);
            }
        } finally {
            // Release the camera when the OpMode stops.
            portal.close();
        }
    }
}
