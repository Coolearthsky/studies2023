package org.team100.lib.storage;

import java.util.AbstractMap;
import java.util.Collections;
import java.util.Map.Entry;
import java.util.NavigableMap;
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
    private final int capacity;
    private final NavigableMap<Long, Entry<Double, Value>> record;
    private final NavigableMap<Double, Entry<Long, Value>> valid;
    private int size;

    public BitemporalBuffer(int capacity) {
        this.capacity = capacity;
        size = 0;
        record = new ConcurrentSkipListMap<Long, Entry<Double, Value>>();
        valid = new ConcurrentSkipListMap<Double, Entry<Long, Value>>();
    }

    /**
     * @param recordTime represents the system time the value was written, expected
     *                   to be FPGATime.
     * @param validTime  represents the real-world time the value describes, in
     *                   seconds. the most-recent entries are retained, up to the
     *                   capacity.
     * @param value      value to store
     */
    public synchronized void put(long recordTime, double validTime, Value value) {
        // fix the keys to avoid overwriting anything.
        // these are tiny increments, won't affect the consumer.
        // synchronized so these increments are applied consistently.
        while (record.containsKey(recordTime)) {
            recordTime++; // add one microsecond
        }
        while (valid.containsKey(validTime)) {
            validTime = Math.nextUp(validTime); // add smallest possible double
        }
        record.put(recordTime, new AbstractMap.SimpleImmutableEntry<>(validTime, value));
        valid.put(validTime, new AbstractMap.SimpleImmutableEntry<>(recordTime, value));
        if (++size > capacity) {
            Entry<Double, Entry<Long, Value>> validEntry = valid.pollFirstEntry();
            Long key = validEntry.getValue().getKey();
            if (record.remove(key) == null) {
                throw new IllegalStateException("This should never happen: missing record: " + key);
            }
            --size;
        }
    }

    public NavigableMap<Long, Entry<Double, Value>> recordTailMap(long tt) {
        return Collections.unmodifiableNavigableMap(record.tailMap(tt, true));
    }

    public NavigableMap<Double, Entry<Long, Value>> validTailMap(double vt) {
        return Collections.unmodifiableNavigableMap(valid.tailMap(vt, true));
    }

    /** Find the entry for the greatest key less than vt. */
    public Entry<Double,Entry<Long,Value>> validFloorEntry(double vt) {
        return valid.floorEntry(vt);
    }

    public int size() {
        return size;
    }
}
