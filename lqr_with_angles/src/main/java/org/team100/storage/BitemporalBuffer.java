package org.team100.storage;

import java.util.AbstractMap;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.SortedMap;
import java.util.concurrent.ConcurrentSkipListMap;

/**
 * Bitemporal Buffer
 * 
 * Implements the Temporal Database concepts of "valid time" and "record
 * time" for point-in-time measurements. Valid time represents the real-world
 * instant that an item refers to. Record time represents the instant that
 * a data record was added to the buffer. Consumers are expected to use
 * record time to fetch new data, which might refer to valid times
 * arbitrarily long ago.
 * 
 * For example, this buffer could be used to store measurements with various
 * latencies. A position measurement might be available almost instantly,
 * whereas a velocity measurement might involve some averaging over time.
 * 
 * An observer might make periodic state updates by consuming measurements, and
 * when out-of-sequence measurements are discovered, the observer would "roll
 * back" to a state earlier than the newly-arriving but older-representing
 * measurement, and then reapply all the updates since then.
 * 
 * Since the rio measures system time as a long and WPILib measures real time as
 * a double, that's what we use here.
 * 
 * https://en.wikipedia.org/wiki/Temporal_database
 * https://www.mathworks.com/help/fusion/ug/handle-out-of-sequence-measurements-with-filter-retrodiction.html
 * 
 *
 */
public class BitemporalBuffer<Value> {
    NavigableMap<Long, Entry<Double, Value>> record = new ConcurrentSkipListMap<Long, Entry<Double, Value>>();
    NavigableMap<Double, Entry<Long, Value>> valid = new ConcurrentSkipListMap<Double, Entry<Long, Value>>();

    public synchronized void put(long tt, double vt, Value v) {
        // fix the keys to avoid overwriting anything.
        // these are tiny increments, won't affect the consumer.
        // synchronized so these increments are applied consistently.
        while (record.containsKey(tt)) {
            tt++; // add one microsecond
        }
        while (valid.containsKey(vt)) {
            vt = Math.nextUp(vt); // add one epsilon
        }
        record.put(tt, new AbstractMap.SimpleImmutableEntry<>(vt, v));
        valid.put(vt, new AbstractMap.SimpleImmutableEntry<>(tt, v));
    }

    public SortedMap<Long, Entry<Double, Value>> recordTailMap(long tt) {
        return record.tailMap(tt);
    }

    public SortedMap<Double, Entry<Long, Value>> validTailMap(double vt) {
        return valid.tailMap(vt);
    }
}
