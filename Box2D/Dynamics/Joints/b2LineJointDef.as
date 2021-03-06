package Box2D.Dynamics.Joints
{
	import Box2D.Dynamics.b2Body;
	import Box2D.Common.b2Vec2;
	
	/// Line joint definition. This requires defining a line of
	/// motion using an axis and an anchor point. The definition uses local
	/// anchor points and a local axis so that the initial configuration
	/// can violate the constraint slightly. The joint translation is zero
	/// when the local anchor points coincide in world space. Using local
	/// anchors and a local axis helps when saving and loading a game.
	public class b2LineJointDef extends b2JointDef
	{
		public function b2LineJointDef()
		{
			type = b2Joint.e_lineJoint;
			localAnchorA.SetZero();
			localAnchorB.SetZero();
			localAxisA.Set(1.0, 0.0);
			enableLimit = false;
			lowerTranslation = 0.0;
			upperTranslation = 0.0;
			enableMotor = false;
			maxMotorForce = 0.0;
			motorSpeed = 0.0;
		}

		/// Initialize the bodies, anchors, axis, and reference angle using the world
		/// anchor and world axis.
		//void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor, const b2Vec2& axis);

		/// The local anchor point relative to body1's origin.
		public var localAnchorA:b2Vec2 = new b2Vec2 ();

		/// The local anchor point relative to body2's origin.
		public var localAnchorB:b2Vec2 = new b2Vec2 ();

		/// The local translation axis in body1.
		public var localAxisA:b2Vec2 = new b2Vec2 ();

		/// Enable/disable the joint limit.
		public var enableLimit:Boolean;

		/// The lower translation limit, usually in meters.
		public var lowerTranslation:Number;

		/// The upper translation limit, usually in meters.
		public var upperTranslation:Number;

		/// Enable/disable the joint motor.
		public var enableMotor:Boolean;

		/// The maximum motor torque, usually in N-m.
		public var maxMotorForce:Number;

		/// The desired motor speed in radians per second.
		public var motorSpeed:Number;
		
		//...
		public function Initialize(b1:b2Body, b2:b2Body, anchor:b2Vec2, axis:b2Vec2):void
		{
			bodyA = b1;
			bodyB = b2;
			bodyA.GetLocalPoint_Output(anchor, localAnchorA);
			bodyB.GetLocalPoint_Output(anchor, localAnchorB);
			localAxisA.CopyFrom (bodyA.GetLocalVector(axis));
		}
	} // class
} // package
