package org.team100.storage;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Iterator;
import java.util.Map.Entry;
import java.util.SortedMap;

import org.junit.jupiter.api.Test;

public class BitemporalBufferTest {

    @Test
    public void testTailMapInclusion() {
        BitemporalBuffer<String> buf = new BitemporalBuffer<>();
        buf.put(0l, 0.0, "hello");
        SortedMap<Long, Entry<Double, String>> s1 = buf.transactionTailMap(0l);
        assertEquals(1, s1.size());
        assertEquals("hello", s1.get(0l).getValue());
        SortedMap<Double, Entry<Long, String>> s2 = buf.validTailMap(0l);
        assertEquals(1, s2.size());
        assertEquals("hello", s2.get(0.0).getValue());
    }

    @Test
    public void testTailMapExclusion() {
        BitemporalBuffer<String> buf = new BitemporalBuffer<>();
        buf.put(0l, 0.0, "hello");
        SortedMap<Long, Entry<Double, String>> s3 = buf.transactionTailMap(1l);
        assertEquals(0, s3.size());
        SortedMap<Double, Entry<Long, String>> s4 = buf.validTailMap(1l);
        assertEquals(0, s4.size());
    }

    @Test
    public void testDuplicateKeys() {
        BitemporalBuffer<String> buf = new BitemporalBuffer<>();
        // add duplicate keys
        buf.put(0l, 0.0, "hello");
        buf.put(0l, 0.0, "duplicate");

        SortedMap<Long, Entry<Double, String>> s1 = buf.transactionTailMap(0l);
        assertEquals(2, s1.size());
        Iterator<Entry<Long,Entry<Double,String>>> transactions = s1.entrySet().iterator();

        // the first entry is here as entered
        Entry<Long,Entry<Double,String>> transactionEntry = transactions.next();
        assertEquals(0l, transactionEntry.getKey());
        Entry<Double, String> transactionValue = transactionEntry.getValue();
        assertEquals(0.0, transactionValue.getKey());
        assertEquals("hello", transactionValue.getValue());

        // the second entry has incremented keys
        transactionEntry = transactions.next();
        assertEquals(1l, transactionEntry.getKey());
        transactionValue = transactionEntry.getValue();
        assertEquals(4.9E-324, transactionValue.getKey());
        assertEquals("duplicate", transactionValue.getValue());

        // the keys are the same in both maps
        SortedMap<Double, Entry<Long, String>> s2 = buf.validTailMap(0l);
        assertEquals(2, s2.size());
        Iterator<Entry<Double,Entry<Long, String>>> valids = s2.entrySet().iterator();

        Entry<Double, Entry<Long, String>> validEntry = valids.next();
        assertEquals(0.0, validEntry.getKey());
        Entry<Long, String> validValue = validEntry.getValue();
        assertEquals(0l, validValue.getKey());
        assertEquals("hello", validValue.getValue());

        validEntry = valids.next();
        assertEquals(4.9E-324, validEntry.getKey());
        validValue = validEntry.getValue();
        assertEquals(1l, validValue.getKey());
        assertEquals("duplicate", validValue.getValue());
    }

}
