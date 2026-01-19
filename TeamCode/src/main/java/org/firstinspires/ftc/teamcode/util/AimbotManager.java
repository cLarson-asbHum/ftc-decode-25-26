package org.firstinspires.ftc.teamcode.util;

import java.io.BufferedInputStream;
import java.io.Closeable;
import java.io.IOException;
import java.io.InputStream;
import java.util.Collection;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.ballistics.ArcFunction;
import org.firstinspires.ftc.teamcode.ballistics.BallisticArc;
import org.firstinspires.ftc.teamcode.ballistics.BallisticArcSelection;
import org.firstinspires.ftc.teamcode.ballistics.BallisticArcSelection.Criterion;
import org.firstinspires.ftc.teamcode.ballistics.BallisticFileIo;
import org.firstinspires.ftc.teamcode.hardware.subsystem.FlywheelTubeShooter;
import org.firstinspires.ftc.teamcode.hardware.subsystem.PivotSubsystem;
import org.firstinspires.ftc.teamcode.res.R;
import org.firstinspires.ftc.teamcode.util.Util;

public class AimbotManager implements Closeable {
    public static final int MAJOR_ARCS_INCREMENT = 250;
    public static final int MINOR_ARCS_INCREMENT = 50;
    public static final String MINOR_INDICATOR = ".";

    public static final double ANGLE_TOLERANCE = Math.toRadians(0.25);
    public static final double SPEED_TOLERANCE = 0.5;

    private InputStream streamReference = null; // Used only to close the stream.
    private Thread readFiles = null;
    private String readStatus = "";
    private int parsedArcs = 0;
    private volatile boolean inProgress = false;

    private final FlywheelTubeShooter shooter;
    private final PivotSubsystem pivot;
    private volatile BallisticArcSelection selection;

    public AimbotManager(FlywheelTubeShooter shooter, PivotSubsystem pivot, BallisticArcSelection selection) {
        this.shooter = shooter;
        this.pivot = pivot;
        this.selection = selection;
    }
    
    public AimbotManager(FlywheelTubeShooter shooter, PivotSubsystem pivot) {
        this(shooter, pivot, null);
    }

    /**
     * Reads the given Android resource for all the arcs. Neither syntax checking nor
     * bounds checking is done on the contents of the arcs file. When done, the 
     * manager's selection will no longer be null.
     * 
     * The reading of the file is done in a separate thread. To interrupt the thread,
     * see `close()`. To check whether the thread has completed, see `isInitialized()`.
     * To get the resulting ballistic selection, see `getSelection()`.
     * 
     * @param resourceId The generated id from the R class.
     * @param filter Returns true if an arc should be kept. 
     * @param telemetry Where to log. If null, then nothing is printed
     * @throws IOException If an error occurs when reading or closing the resource stream
     */
    public synchronized void init(int resourceId, ArcFunction filter, final Telemetry telemetry ) throws IOException{
        // Canceling if another initialization is in progress
        if(inProgress) {
            return;
        }

        inProgress = true;
        selection = null;

        // FIXME: AppUtil prevents unit testing because of RobotCore lib dependence.
        // Reading out the entire stream
        final int BUFFER_SIZE = Float.BYTES * 4 * 1000; // A size of 1000 arc
        final InputStream stream = new BufferedInputStream( 
            AppUtil
                .getDefContext()
                .getResources()
                .openRawResource(resourceId), 
            BUFFER_SIZE 
        );

        streamReference = stream; // Used only to close the stream in stop()

        // Converting it into something useful
        if(!stream.markSupported()) {
            stream.close();
            throw new IOException("Mark is not supported by the arcs.bin input sream.");
        }

        final ArcFunction wrappedFilter = telemetry == null ? filter : countArc(telemetry, filter);
        readFiles = new Thread(() -> {
            try {
                // Getting the arcs
                final Collection<BallisticArc> arcs = BallisticFileIo.readArcs(stream, wrappedFilter);
                
                // Ending if we have been interrupted
                if(Thread.interrupted()) {
                    stream.close();
                    return;
                }
                
                // Creating the selection
                if(telemetry != null) {
                    telemetry.clear();
                    telemetry.addData("Status", "Sorting the arcs...");
                    telemetry.update();
                }

                selection = new BallisticArcSelection(arcs);

                // Cleaning up
                stream.close();
                streamReference = null;
                readStatus = null;
                readFiles = null;
                inProgress = false;
            } catch(IOException exc) {
                throw new RuntimeException(exc);
            }
        });
        readFiles.start();
    }

    /**
     * Reads the given Android resource for all the arcs. Neither syntax checking nor
     * bounds checking is done on the contents of the arcs file. When done, the 
     * manager's selection will no longer be null.
     * 
     * The reading of the file is done in a separate thread. To interrupt the thread,
     * see `close()`. To check whether the thread has completed, see `isInitialized()`.
     * To get the resulting ballistic selection, see `getSelection()`.
     * 
     * @param resourceId The generated id from the R class.
     * @param filter Returns true if an arc should be kept. 
     * @throws IOException If an error occurs when reading or closing the resource stream
     */
    public synchronized void init(int resourceId, ArcFunction filter) throws IOException {
        init(resourceId, filter, null);
    }

    /**
     * Checks whether the current selection is null. This is the case if no 
     * selection was provided at construction and `init()` has not been called,
     * or if `init()` has just been called but has not finished.
     * 
     * @return Whether the selection of ballistic arcs has been initialized
     */
    public synchronized boolean isInitialized() {
        return selection != null;
    }

    /**
     * Determines whether `init()` has been called but has not finished. If
     * this method is true, it is guranteeds that the selection will be null
     * and that `isInitialized()` will be false.
     * 
     * @return Whether an initalization of the selection is in progress.
     */
    public synchronized boolean isInitInProgress() {
        return inProgress;
    }

    /**
     * Returns the ballistic arcs that this manager uses. This is null if
     * none was provided at construction and `init()` has not finished.
     * 
     * @return All the ballistic arcs that are used when selecting an arc
     */
    public BallisticArcSelection getSelection() {
        return this.selection;
    }
    
    /**
     * Gets an arc that best matches the given distance. A efw other states are 
     * selected for, such as minimum speed and maximum angle. If no arc is found because
     * the distance is out of range, then this returns the closest bound
     * 
     * The shooter and pivot are not updated by this method.
     * 
     * @param targetDist - The target to reach, in inches
     * @param tolerance - Tolerance the arc can be within the distance, in inches
     * @return The arc that best matches, or null if none was found (incredibly rare!).
     */
    public BallisticArc selectArc(double targetDist, double tolerance) {
        final double min = Criterion.DISTANCE.of(selection.minDistance().first());
        final double max = Criterion.DISTANCE.of(selection.maxDistance().first());
        
        // Sending this data to the shooter and pivot
        return selection
            .withinDistance(Util.clamp(min, targetDist, max), tolerance)
            .minSpeed(SPEED_TOLERANCE)  // First Tiebreaker,  so as to make charging easiest
            .maxAngle(ANGLE_TOLERANCE)  // Second tiebreaker, so as to maximize tolerance
            .maxDistance()             // Third Tiebreaker,  to get as far as possible
            .first();
            // .minAngle(ANGLE_TOLERANCE)  // Second Tiebreaker, so as to avoid losing speed at high angles
    }
    
    /**
     * Gives the shooter and the pivot subsystems the arc's speed and angle 
     * respectively.
     * 
     * @param arc What angle and speed to use for the shooter.
     */
    public void followArc(BallisticArc arc) {
        pivot.runToAngle(Criterion.ANGLE.of(arc), ANGLE_TOLERANCE);
        shooter.charge(Criterion.SPEED.of(arc), false);
    }

    @Override
    public synchronized void close() throws IOException {
        if(readFiles != null) { 
            readFiles.interrupt();
        }

        if(streamReference != null) {
            streamReference.close();
        }
        
        streamReference = null;
        readStatus = null;
        readFiles = null;
    }    
    
    private ArcFunction countArc(Telemetry telemetry, final ArcFunction filter) {
        return arc -> {
            boolean update = false;
            parsedArcs++;

            if(parsedArcs % MINOR_ARCS_INCREMENT == 0) {
                readStatus += MINOR_INDICATOR;
                update = true;
            }

            if(parsedArcs % MAJOR_ARCS_INCREMENT == 0) {
                readStatus += parsedArcs;
                update = true;
            }

            if(update) {
                telemetry.addData("Status", "Reading ballistic arcs...");
                telemetry.addLine(readStatus);
                telemetry.update();
            }

            return filter.apply(arc);
        };
    }

}