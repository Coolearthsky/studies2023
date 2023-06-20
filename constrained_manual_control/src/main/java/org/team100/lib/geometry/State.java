package org.team100.lib.geometry;



import org.team100.lib.util.CSVWritable;
import org.team100.lib.util.Interpolable;

/** This is cut and paste from 254. */
public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    S add(S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}