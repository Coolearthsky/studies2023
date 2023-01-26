package org.dyn4j.samples;

import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.joint.AngleJoint;
import org.dyn4j.dynamics.joint.DistanceJoint;
import org.dyn4j.dynamics.joint.RevoluteJoint;
import org.dyn4j.dynamics.joint.WeldJoint;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.samples.framework.SimulationBody;
import org.dyn4j.samples.framework.SimulationFrame;

public class Arm extends SimulationFrame {
	private static final long MASK_ALL = Long.MAX_VALUE;
	private static final long MASK_NONE = 0;
	private static final long CAT_NORMAL = 1;
	private static final long CAT_MAGIC = 2;
	private static final CategoryFilter normalFilter = new CategoryFilter(CAT_NORMAL, MASK_ALL ^ CAT_MAGIC);
	private static final CategoryFilter magicFilter = new CategoryFilter(CAT_MAGIC, MASK_NONE);

	public Arm() {
		super("2-DOF Arm Counterbalance");
	}

	protected void initializeWorld() {
		SimulationBody frame = new SimulationBody();
		BodyFixture frameFixture = frame.addFixture(Geometry.createRectangle(0.5, 10.0));
		frameFixture.setFilter(normalFilter);
		frame.translate(new Vector2(0.0, 0.0));
		frame.setMass(MassType.INFINITE);
		world.addBody(frame);

		SimulationBody humerus = new SimulationBody();
		BodyFixture humerusFilter = humerus.addFixture(Geometry.createRectangle(0.5, 5.0), 2.0, 0.0, 0.0);
		humerusFilter.setFilter(normalFilter);
		humerus.translate(new Vector2(0.0, 2.5));
		humerus.setMass(MassType.NORMAL);
		world.addBody(humerus);

		RevoluteJoint<SimulationBody> shoulder = new RevoluteJoint<SimulationBody>(
				frame, humerus, new Vector2(0.0, 0.0));
		world.addJoint(shoulder);

		DistanceJoint<SimulationBody> deltoid = new DistanceJoint<SimulationBody>(
				frame, humerus, new Vector2(0.0, 2.5), new Vector2(0.0, 2.5));
		deltoid.setSpringStiffness(184);
		deltoid.setSpringEnabled(true);
		deltoid.setRestDistance(0.0);
		world.addJoint(deltoid);

		SimulationBody ulna = new SimulationBody();
		BodyFixture ulnaFixture = ulna.addFixture(Geometry.createRectangle(0.5, 5.0), 2.0, 0.0, 0.0);
		ulnaFixture.setFilter(normalFilter);
		ulna.translate(new Vector2(0.0, 7.5));
		ulna.setMass(MassType.NORMAL);
		world.addBody(ulna);

		RevoluteJoint<SimulationBody> elbow = new RevoluteJoint<SimulationBody>(
				humerus, ulna, new Vector2(0.0, 5.0));
		world.addJoint(elbow);

		SimulationBody ulnaLinkage = new SimulationBody();
		BodyFixture ulnaLinkageFixture = ulnaLinkage.addFixture(Geometry.createRectangle(0.5, 5.0), 2.0, 0.0, 0.0);
		ulnaLinkageFixture.setFilter(magicFilter);
		ulnaLinkage.translate(new Vector2(0.0, -2.5));
		ulnaLinkage.setMass(MassType.NORMAL);
		world.addBody(ulnaLinkage);

		RevoluteJoint<SimulationBody> ulnaLinkagePivot = new RevoluteJoint<SimulationBody>(
				ulnaLinkage, frame, new Vector2(0.0, 0.0));
		world.addJoint(ulnaLinkagePivot);

		AngleJoint<SimulationBody> beltConstraint = new AngleJoint<SimulationBody>(
				ulna, ulnaLinkage);
		world.addJoint(beltConstraint);

		DistanceJoint<SimulationBody> bicep = new DistanceJoint<SimulationBody>(
				frame, ulnaLinkage, new Vector2(0.0, -2.5), new Vector2(0.0, -2.5));
		bicep.setSpringStiffness(150);
		bicep.setSpringEnabled(true);
		bicep.setRestDistance(0.0);
		world.addJoint(bicep);

		SimulationBody hand = new SimulationBody();
		BodyFixture handFixture = hand.addFixture(Geometry.createRectangle(2.0, 2.0), 4.0, 0.0, 0.0);
		handFixture.setFilter(magicFilter);
		hand.translate(new Vector2(0.0, 11.0));
		hand.setMass(MassType.NORMAL);
		world.addBody(hand);

		WeldJoint<SimulationBody> wrist = new WeldJoint<SimulationBody>(
				hand, ulna, new Vector2(0, 10.0));
		world.addJoint(wrist);
	}

	public static void main(String[] args) {
		Arm simulation = new Arm();
		simulation.run();
	}
}