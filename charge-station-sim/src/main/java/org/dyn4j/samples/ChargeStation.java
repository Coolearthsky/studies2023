package org.dyn4j.samples;

import java.awt.Color;

import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.joint.RevoluteJoint;
import org.dyn4j.dynamics.joint.WeldJoint;
import org.dyn4j.geometry.Circle;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.samples.framework.Camera;
import org.dyn4j.samples.framework.SimulationBody;
import org.dyn4j.samples.framework.SimulationFrame;

public class ChargeStation extends SimulationFrame {
	// private static final long serialVersionUID = 5663760293144882635L;

	private static final long ALL = Long.MAX_VALUE;
	private static final long HINGE_STRAP = 1;
	private static final long OTHER = 2;
	private static final long HINGE_MOUNT = 4;
	private static final CategoryFilter hingeStrapFilter = new CategoryFilter(HINGE_STRAP, HINGE_MOUNT);
	private static final CategoryFilter allFilter = new CategoryFilter(OTHER, ALL);
	private static final CategoryFilter hingeMountFilter = new CategoryFilter(HINGE_MOUNT, ALL);

	public ChargeStation() {
		super("Graphics2D Example");
	}

	private static double m(double inches) {
		return inches / 39.37;
	}

	protected void initializeWorld() {

		this.world.getSettings().setLinearTolerance(0.001); // instead of 0.005
		// units are inches, as specified in
		// https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/TeamVersions/Drawings/TE-23301-HalfChargeStation-Dwg.pdf

		// since dyn4j likes meters, all the numbers here are converted to meters

		// floor
		SimulationBody floor = new SimulationBody();
		BodyFixture floorFixture = floor.addFixture(new Rectangle(m(160.0), m(1.0)));
		floorFixture.setFilter(allFilter);
		floor.setMass(MassType.INFINITE);
		floor.translate(m(0.0), m(-0.5)); // so floor surface is at zero
		this.world.addBody(floor);

		// TE-23305 (hinge base)
		SimulationBody te23305 = new SimulationBody();
		BodyFixture te23305Fixture = te23305.addFixture(new Rectangle(m(18), m(0.47)));
		te23305Fixture.setFilter(allFilter);
		te23305.setMass(MassType.INFINITE);
		te23305.translate(m(0.0), m(0.47 / 2));
		this.world.addBody(te23305);

		// TE-23317 (hinge stand)
		SimulationBody te23317 = new SimulationBody();
		BodyFixture te23317Fixture = te23317.addFixture(new Rectangle(m(1.5), m(3.94)));
		te23317Fixture.setFilter(allFilter);
		te23317.setMass(MassType.INFINITE);
		te23317.translate(m(-1.75), m(3.94 / 2 + 0.47));
		this.world.addBody(te23317);

		// TE-23317 (hinge stand)
		SimulationBody te23317_2 = new SimulationBody();
		BodyFixture te23317_2Fixture = te23317_2.addFixture(new Rectangle(m(1.5), m(3.94)));
		te23317_2Fixture.setFilter(allFilter);
		te23317_2.setMass(MassType.INFINITE);
		te23317_2.translate(m(1.75), m(3.94 / 2 + 0.47));
		this.world.addBody(te23317_2);

		// TE-23318 (hinge lintel)
		SimulationBody te23318 = new SimulationBody();
		BodyFixture te23318Fixture = te23318.addFixture(new Rectangle(m(5.0), m(1.5)));
		te23318Fixture.setFilter(allFilter);
		te23318.setMass(MassType.INFINITE);
		te23318.translate(m(0), m(1.5 / 2 + 3.95 + 0.47));
		this.world.addBody(te23318);

		// TE-23316 (hinge support)
		SimulationBody te23316 = new SimulationBody();
		BodyFixture te23316Fixture = te23316.addFixture(new Rectangle(m(3.5), m(5.44)));
		te23316Fixture.setFilter(allFilter);
		te23316.setMass(MassType.INFINITE);
		te23316.translate(m(2.5 + 0.75 + 3.5 / 2), m(5.44 / 2 + 0.47));
		this.world.addBody(te23316);

		// TE-23311 (fixed hinge core)
		SimulationBody te23311 = new SimulationBody();
		BodyFixture te23311Fixture = te23311.addFixture(new Rectangle(m(6.63), m(1.0)));
		te23311Fixture.setFilter(allFilter);
		te23311.setMass(MassType.INFINITE);
		te23311.translate(m(2.5 + 0.75 + 3.5 - 6.63 / 2), m(1.0 / 2 + 5.44 + 0.47));
		this.world.addBody(te23311);

		// TE-23310 (hinge strap)
		SimulationBody te23310 = new SimulationBody();
		BodyFixture te23310Fixture = te23310.addFixture(new Rectangle(m(6.0), m(1.0)));
		te23310Fixture.setFilter(hingeStrapFilter); // collide with nothing
		// depth is ~0.02m, density is about 500 kg/m^3 so 10 kg/m^2
		te23310Fixture.setDensity(100);
		te23310.setMass(MassType.NORMAL); // TODO: density?
		te23310.translate(m(2.5 + 0.75 + 3.5 - 4.13 - 5.25 / 2), m(1.0 / 2 + 5.44 + 0.47));
		this.world.addBody(te23310);

		// fixed pin
		RevoluteJoint<SimulationBody> fixedPin = new RevoluteJoint<SimulationBody>(te23311, te23310,
				new Vector2(m(2.5 + 0.75 + 3.5 - 4.13), m(1.0 / 2 + 5.44 + 0.47)));
		this.world.addJoint(fixedPin);

		// TE-23311_2 (moving hinge core)
		SimulationBody te23311_2 = new SimulationBody();
		BodyFixture te23311_2Fixture = te23311_2.addFixture(new Rectangle(m(6.63), m(1.0)));
		te23311_2Fixture.setFilter(allFilter);
		te23311_2Fixture.setDensity(100);
		te23311_2.setMass(MassType.NORMAL);
		te23311_2.translate(m(2.5 + 0.75 + 3.5 - 4.13 - 5.25 - 4.13 + 6.63 / 2), m(1.0 / 2 + 5.44 + 0.47));
		this.world.addBody(te23311_2);

		// moving pin
		RevoluteJoint<SimulationBody> movingPin = new RevoluteJoint<SimulationBody>(te23310, te23311_2,
				new Vector2(m(2.5 + 0.75 + 3.5 - 4.13 - 5.25), m(1.0 / 2 + 5.44 + 0.47)));
		this.world.addJoint(movingPin);

		// TE-23316_2 (moving hinge support)
		SimulationBody te23316_2 = new SimulationBody();
		BodyFixture te23316_2Fixture = te23316_2.addFixture(new Rectangle(m(3.5), m(1.5)));
		te23316_2Fixture.setFilter(allFilter);
		te23316_2Fixture.setDensity(200); // 4cm depth
		te23316_2.setMass(MassType.NORMAL);
		te23316_2.translate(m(2.5 + 0.75 + 3.5 - 4.13 - 5.25 - 4.13 + 3.5 / 2), m(1.5 / 2 + 1.0 + 5.44 + 0.47));
		this.world.addBody(te23316_2);

		WeldJoint<SimulationBody> movingHingeSupportToMovingHingeCore = new WeldJoint<SimulationBody>(te23316_2,
				te23311_2,
				new Vector2(m(2.5 + 0.75 + 3.5 - 4.13 - 5.25 - 4.13 + 3.5 / 2), m(1.5 + 1.0 + 5.44 + 0.47)));
		this.world.addJoint(movingHingeSupportToMovingHingeCore);

		// TE-23302 (top)
		SimulationBody te23302 = new SimulationBody();
		BodyFixture te23302Fixture = te23302.addFixture(new Rectangle(m(48), m(0.72)));
		te23302Fixture.setFilter(allFilter);
		te23302Fixture.setDensity(610); // 1.2m depth
		te23302.setMass(MassType.NORMAL);
		te23302.translate(m(0), m(0.72 / 2 + 1.5 + 1.0 + 5.44 + 0.47));
		this.world.addBody(te23302);

		WeldJoint<SimulationBody> topToMovingHingeSupport = new WeldJoint<SimulationBody>(te23302,
				te23316_2,
				new Vector2(m(3.5 / 2 - 1.5 - 3.5 / 2), m(1.5 + 1.0 + 5.44 + 0.47)));
		this.world.addJoint(topToMovingHingeSupport);

		// TE-23313 (hinge mount)
		SimulationBody te23313 = new SimulationBody();
		BodyFixture te23313Fixture = te23313.addFixture(new Rectangle(m(3.5), m(1.5)));
		te23313Fixture.setFilter(hingeMountFilter);
		te23313Fixture.setDensity(610); // 1.2m depth
		te23313.setMass(MassType.NORMAL);
		te23313.translate(m(0), m(1.5 / 2 + 1.0 + 5.44 + 0.47));
		this.world.addBody(te23313);

		WeldJoint<SimulationBody> topToHingeMount = new WeldJoint<SimulationBody>(te23302,
				te23313,
				new Vector2(m(0), m(1.5 + 1.0 + 5.44 + 0.47)));
		this.world.addJoint(topToHingeMount);

		// TE-23303_1 (left leaf)
		SimulationBody te23303_1 = new SimulationBody();
		BodyFixture te23303_1Fixture = te23303_1.addFixture(new Rectangle(m(14.25), m(0.72)));
		te23303_1Fixture.setFilter(allFilter);
		te23303_1Fixture.setDensity(610); // 1.2m depth
		te23303_1.setMass(MassType.NORMAL);
		// this is a guess for the hinge size
		te23303_1.translate(m(-24 - 0.72 - 14.25 / 2), m(0.72 / 2 + 1.5 + 1.0 + 5.44 + 0.47));
		this.world.addBody(te23303_1);

		// left hinge
		RevoluteJoint<SimulationBody> leftHinge = new RevoluteJoint<SimulationBody>(te23303_1, te23302,
				new Vector2(m(-24 - 0.72 / 2), m(1.5 + 1.0 + 5.44 + 0.47)));
		this.world.addJoint(leftHinge);

		// TE-23303_2 (right leaf)
		SimulationBody te23303_2 = new SimulationBody();
		BodyFixture te23303_2Fixture = te23303_2.addFixture(new Rectangle(m(14.25), m(0.72)));
		te23303_2Fixture.setFilter(allFilter);
		te23303_2Fixture.setDensity(610); // 1.2m depth
		te23303_2.setMass(MassType.NORMAL);
		// this is a guess for the hinge size
		te23303_2.translate(m(24 + 0.72 + 14.25 / 2), m(0.72 / 2 + 1.5 + 1.0 + 5.44 + 0.47));
		this.world.addBody(te23303_2);

		// right hinge
		RevoluteJoint<SimulationBody> rightHinge = new RevoluteJoint<SimulationBody>(te23303_2, te23302,
				new Vector2(m(24 + 0.72 / 2), m(1.5 + 1.0 + 5.44 + 0.47)));
		this.world.addJoint(rightHinge);

		// left limiter
		SimulationBody te23315_1 = new SimulationBody();
		BodyFixture te23315_1Fixture = te23315_1.addFixture(new Rectangle(m(3.5), m(3.81)));
		te23315_1Fixture.setFilter(allFilter);
		te23315_1Fixture.setDensity(610);
		te23315_1.setMass(MassType.NORMAL);
		te23315_1.translate(m(-41.25 / 2 + 3.5 / 2), m(-3.81 / 2 + 1.5 + 1.0 + 5.44 + 0.47));
		this.world.addBody(te23315_1);
		WeldJoint<SimulationBody> limiter1ToTop = new WeldJoint<SimulationBody>(te23315_1,
				te23302,
				new Vector2(m(-41.25 / 2 + 3.5 / 2), m(1.5 + 1.0 + 5.44 + 0.47)));
		this.world.addJoint(limiter1ToTop);

		// right limiter
		SimulationBody te23315_2 = new SimulationBody();
		BodyFixture te23315_2Fixture = te23315_2.addFixture(new Rectangle(m(3.5), m(3.81)));
		te23315_2Fixture.setFilter(allFilter);
		te23315_2Fixture.setDensity(610);
		te23315_2.setMass(MassType.NORMAL);
		te23315_2.translate(m(41.25 / 2 - 3.5 / 2), m(-3.81 / 2 + 1.5 + 1.0 + 5.44 + 0.47));
		this.world.addBody(te23315_2);
		WeldJoint<SimulationBody> limiter2ToTop = new WeldJoint<SimulationBody>(te23315_2,
				te23302,
				new Vector2(m(41.25 / 2 - 3.5 / 2), m(1.5 + 1.0 + 5.44 + 0.47)));
		this.world.addJoint(limiter2ToTop);

		// left hinge limiter
		SimulationBody te23308_1 = new SimulationBody();
		BodyFixture te23308_1Fixture = te23308_1.addFixture(new Rectangle(m(4.25), m(0.47)));
		te23308_1Fixture.setFilter(allFilter);
		te23308_1Fixture.setDensity(610);
		te23308_1.setMass(MassType.NORMAL);
		te23308_1.translate(m(-41.25 / 2 - 4.25 / 2), m(-0.47 / 2 - 1.5 - 0.47 + 1.5 + 1.0 + 5.44 + 0.47));
		this.world.addBody(te23308_1);
		WeldJoint<SimulationBody> hingeLimiter1ToTop = new WeldJoint<SimulationBody>(te23308_1,
				te23302,
				new Vector2(m(-41.25 / 2 - 4.25 / 2), m(1.5 + 1.0 + 5.44 + 0.47)));
		this.world.addJoint(hingeLimiter1ToTop);

		// right hinge limiter
		SimulationBody te23308_2 = new SimulationBody();
		BodyFixture te23308_2Fixture = te23308_2.addFixture(new Rectangle(m(4.25), m(0.47)));
		te23308_2Fixture.setFilter(allFilter);
		te23308_2Fixture.setDensity(610);
		te23308_2.setMass(MassType.NORMAL);
		te23308_2.translate(m(41.25 / 2 + 4.25 / 2), m(-0.47 / 2 - 1.5 - 0.47 + 1.5 + 1.0 + 5.44 + 0.47));
		this.world.addBody(te23308_2);
		WeldJoint<SimulationBody> hingeLimiter2ToTop = new WeldJoint<SimulationBody>(te23308_2,
				te23302,
				new Vector2(m(41.25 / 2 + 4.25 / 2), m(1.5 + 1.0 + 5.44 + 0.47)));
		this.world.addJoint(hingeLimiter2ToTop);

		double robotFrameDistance = 52; // inches away from center

		// robot frame
		SimulationBody robotFrame = new SimulationBody();
		BodyFixture robotFrameFixture = robotFrame.addFixture(new Rectangle(m(30), m(2)));
		robotFrameFixture.setFilter(allFilter);
		// 50kg total(ish), all in the frame which has area 0.038 m^2
		robotFrameFixture.setDensity(1300);
		robotFrame.setMass(MassType.NORMAL);
		robotFrame.translate(m(robotFrameDistance + 15), m(5));
		this.world.addBody(robotFrame);

		// front wheel
		SimulationBody frontWheel = new SimulationBody();
		BodyFixture frontWheelFixture = frontWheel.addFixture(new Circle(m(2)));
		frontWheelFixture.setFilter(allFilter);
		frontWheelFixture.setDensity(100); // it's all in the frame
		frontWheelFixture.setFriction(1);
		frontWheel.setMass(MassType.NORMAL);
		frontWheel.translate(m(robotFrameDistance + 2.5), m(2.0));
		this.world.addBody(frontWheel);

		// front wheel joint
		RevoluteJoint<SimulationBody> frontWheelJoint = new RevoluteJoint<SimulationBody>(frontWheel, robotFrame,
				new Vector2(m(robotFrameDistance + 2.5), m(2)));
		frontWheelJoint.setMotorEnabled(true);
		frontWheelJoint.setMotorSpeed(1.5); // rad/s
		this.world.addJoint(frontWheelJoint);

		// back wheel
		SimulationBody backWheel = new SimulationBody();
		BodyFixture backWheelFixture = backWheel.addFixture(new Circle(m(2)));
		backWheelFixture.setFilter(allFilter);
		backWheelFixture.setDensity(100);
		backWheelFixture.setFriction(1);
		backWheel.setMass(MassType.NORMAL);
		backWheel.translate(m(robotFrameDistance + 30 - 2.5), m(2.0));
		this.world.addBody(backWheel);

		// back wheel joint
		RevoluteJoint<SimulationBody> backWheelJoint = new RevoluteJoint<SimulationBody>(backWheel, robotFrame,
				new Vector2(m(robotFrameDistance + 30 - 2.5), m(2)));
		backWheelJoint.setMotorEnabled(true);
		backWheelJoint.setMotorSpeed(1.5);
		this.world.addJoint(backWheelJoint);

		// ###############################
		//
		// Case 1: bumper only.
		// lower bumper noodle
		SimulationBody lowerNoodle = new SimulationBody();
		BodyFixture lowerNoodleFixture = lowerNoodle.addFixture(new Circle(m(2.5 / 2)));
		lowerNoodleFixture.setFilter(allFilter);
		lowerNoodleFixture.setDensity(100);
		// lowerNoodleFixture.setFriction(0); // frictionless works, duh
		lowerNoodleFixture.setFriction(0.015); // very very very slippery
		// these do not work
		// lowerNoodleFixture.setFriction(0.03);
		// lowerNoodleFixture.setFriction(0.06);
		// lowerNoodleFixture.setFriction(0.125);
		// lowerNoodleFixture.setFriction(0.25);
		// lowerNoodleFixture.setFriction(0.5);
		// lowerNoodleFixture.setFriction(1);

		lowerNoodle.setMass(MassType.NORMAL);
		lowerNoodle.translate(m(robotFrameDistance - 2.5 / 2 - 0.75), m(2.5 + 2.5 / 2));
		this.world.addBody(lowerNoodle);

		WeldJoint<SimulationBody> lowerBumperJoint = new WeldJoint<SimulationBody>(lowerNoodle, robotFrame,
				new Vector2(m(robotFrameDistance - 0.75), m(2.5 + 2.5 / 2)));
		this.world.addJoint(lowerBumperJoint);

		// upper bumper noodle
		SimulationBody upperNoodle = new SimulationBody();
		BodyFixture upperNoodleFixture = upperNoodle.addFixture(new Circle(m(2.5 / 2)));
		upperNoodleFixture.setFilter(allFilter);
		upperNoodleFixture.setDensity(100);
		upperNoodle.setMass(MassType.NORMAL);
		upperNoodle.translate(m(robotFrameDistance - 2.5 / 2 - 0.75), m(7.5 - 2.5 / 2));
		this.world.addBody(upperNoodle);

		WeldJoint<SimulationBody> upperBumperJoint = new WeldJoint<SimulationBody>(upperNoodle, robotFrame,
				new Vector2(m(robotFrameDistance - 0.75), m(7.5 - 2.5 / 2)));
		this.world.addJoint(upperBumperJoint);

		// ################################
		//
		// Case 2: add a pusher further out and higher
		SimulationBody pusher = new SimulationBody(Color.BLACK);
		BodyFixture pusherFixture = pusher.addFixture(new Circle(m(2.5 / 2)));
		pusherFixture.setFilter(allFilter);
		pusherFixture.setDensity(100);
		pusherFixture.setFriction(0.25); // kinda slippery but not extremely so
		pusher.setMass(MassType.NORMAL);
		pusher.translate(m(robotFrameDistance - 8), m(7));
		this.world.addBody(pusher);

		WeldJoint<SimulationBody> pusherJoint = new WeldJoint<SimulationBody>(pusher, robotFrame,
				new Vector2(m(robotFrameDistance - 8), m(7)));
		this.world.addJoint(pusherJoint);

	}

	@Override
	protected void initializeCamera(Camera camera) {
		super.initializeCamera(camera);
		camera.scale = 45.0;
	}

	public static void main(String[] args) {
		ChargeStation simulation = new ChargeStation();
		simulation.run();
	}
}
