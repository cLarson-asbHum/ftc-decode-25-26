package ballistics;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.util.LinkedList;

public final class BallisticFileIo {
    public static final int METADATA_BYTES = Float.BYTES + Integer.BYTES;

    /**
     * Writes the arcs's points and start velocities to the file.
     */
    public static File appendArc(File file, BallisticArc arc) throws IOException {
        final FileOutputStream writer = new FileOutputStream(file, true);
        try {
            appendArc(writer, arc);
        } catch(IOException exception) {
            writer.close();
            throw exception;
        }

        writer.close();
        return file;
    }

    /**
     * Writes the arcs's points and start velocities using the FileOutputStream. The writer is 
     * not closed by this method.
     */
    public static void appendArc(FileOutputStream writer, BallisticArc arc) throws IOException {
        // Writing the metadata of the points
        writeFloats(writer, (float) arc.getElapsedTime());
        writeInt(writer, arc.size());

        try {
            // Writing the points themselves
            arc.pointIterator().forEachRemaining((point) -> {
                try {
                    writeFloats(writer, (float) point.x, (float) point.y);
                } catch(IOException exception) {
                    throw new InnerException(exception);
                }
            });

            // Writing the velocities
            arc.velIterator().forEachRemaining((vel) -> {
                try {
                    writeFloats(writer, (float) vel.x, (float) vel.y);
                } catch(IOException exception) {
                    throw new InnerException(exception);
                }
            });
        } catch(InnerException exception) {
            throw ((IOException) exception.getCause());
        }
    }

    /**
     * Supplies data for a ballistic arc lazily. This is in contrast to ComputableBallisiticArc,
     * which stores all its data at once upon computation. These, however, cannot be computed
     */
    private static final class BallisticArcStream implements BallisticArc {
        private final FileChannel channel; 
        private final long channelStartPosition;
        private final double elapsed;
        private final int size;

        /**
         * Creates a ballistic arc that sources its data from a file.
         * @param channel The file data. The position must be at the first byte of data 
         * (immediately after the size metadata field)
         * @param elapsed The time surmised by the arc. Returned by `getElapsedTime()`
         * @param pointSize The number of points in the arc. This is equal to the number of 
         * velocities but **not** the number of bytes. 
         */
        BallisticArcStream(FileChannel channel, float elapsed, int pointSize) throws IOException {
            this.elapsed = elapsed;
            this.size = pointSize;
            this.channel = channel;
            this.channelStartPosition = channel.position();
        }

        @Override
        public int size() {
            return this.size;
        }

        @Override
        public Vector getPoint(int i) {
            try {
                final byte[] buf = new byte[2 * Float.BYTES];
                final ByteBuffer bufProxy = ByteBuffer.wrap(buf);
                channel.read(bufProxy, channelStartPosition + buf.length * i);
                return new Vector(
                    Float.intBitsToFloat(intFromBuf(buf, 0)), 
                    Float.intBitsToFloat(intFromBuf(buf, 4))
                ); 
            } catch(IOException exception) {
                throw new InnerException(exception);
            }
        }

        @Override
        public Vector getVel(int i) {
            try {
                final byte[] buf = new byte[2 * Float.BYTES];
                final ByteBuffer bufProxy = ByteBuffer.wrap(buf);
                channel.read(bufProxy, channelStartPosition + buf.length * (i + size));
                return new Vector(
                    Float.intBitsToFloat(intFromBuf(buf, 0)), 
                    Float.intBitsToFloat(intFromBuf(buf, 4))
                ); 
            } catch(IOException exception) {
                throw new InnerException(exception);
            }
        }

        @Override
        public double getElapsedTime() {
            return this.elapsed;
        }

        @Override
        public boolean compute(double unused1, ArcFunction unused2) {
            return false;
        }

        public BallisticArc concretize() {
            return new ImmutableBallisticArc(this);
        }
    }

    private static int unsigned(byte bits) {
        return ((int) bits) << 24 >>> 24;
    }

    private static int intFromBuf(byte[] buf, int start) {
        int result = 0;
        for(int i = 0; i < Integer.BYTES; i++) {
            result = result | (unsigned(buf[start + i]) << (i << 3));
        }
        return result;
    }

    /**
     * Returns all the arcs in the file that satisfy the given condition. 
     */
    public static LinkedList<BallisticArc> readArcs(File file, ArcFunction filter) throws IOException {
        final LinkedList<BallisticArc> result = new LinkedList<>();
        final FileInputStream reader = new FileInputStream(file);
        int position = 0;

        try {
            final int MAX_ITERS = Integer.MAX_VALUE; // Infinite loop prevention 

            // NOTE: This loop is ordinarily broken by a break statement inside the loop
            //       The only reason we are using a for loop is to provide infinite loop 
            //       prevention
            fileParsingLoop:
            for(int iters = 0; iters < MAX_ITERS; iters++) {
                // Gettting the meta data
                final byte[] buf = new byte[METADATA_BYTES];
                position += buf.length;

                if(reader.read(buf) == -1) {
                    // read only returns -1 if the end of the file has been reached.
                    break fileParsingLoop;
                }
                
                final float elapsed = Float.intBitsToFloat(intFromBuf(buf, 0));
                final int size = intFromBuf(buf, 4); // in point vectors, not total point and velocit vecs nor file bytes

                // Creating the ballistic arc stream
                final FileChannel channel = FileChannel.open(file.toPath());
                channel.position(position);
                final BallisticArcStream stream = new BallisticArcStream(channel, elapsed, size);

                // Concretizing the arc and adding it to the result only if it satisfies the filter
                if(filter.apply(stream)) {
                    result.add(stream.concretize());
                }
                channel.close();

                // Going to the next arc
                final int dataByteLength = size * 4 * Float.BYTES;
                reader.skip(dataByteLength);
                position += dataByteLength;
            }
        } catch(IOException | InnerException exception) {
            reader.close();
            throw exception;
        }

        reader.close();
        return result;
    }

    private static void writeFloats(FileOutputStream writer, float... floats) throws IOException {
        // Serializing into a byte buffer
        // Floats are stored little endian
        final byte[] buf = new byte[floats.length * Float.BYTES];

        for(int bufStartIndex = 0; bufStartIndex < buf.length; bufStartIndex += 4) {
            final int bits = Float.floatToIntBits(floats[bufStartIndex >> 2]);
            for(int i = 0; i < Float.BYTES; i++) {
                final int shifter = i << 3; // 8 * i
                buf[bufStartIndex + i] = (byte) ((bits & (0xff << shifter)) >>> shifter);

            }
        }

        // Writing the bytes
        writer.write(buf);
    }

    private static void writeInt(FileOutputStream writer, int bits) throws IOException {
        // Serializing into a byte buffer
        // Floats are stored little endian
        final byte[] buf = new byte[Integer.BYTES];
        for(int i = 0; i < Integer.BYTES; i++) {
            final int shifter = i << 3; // 8 * i
            buf[i] = (byte) ((bits & (0xff << shifter)) >>> shifter);
        }

        // Writing the bytes
        writer.write(buf);
    }
}