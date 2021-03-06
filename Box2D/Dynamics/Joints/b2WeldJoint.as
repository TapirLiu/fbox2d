/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

//#ifndef B2_WELD_JOINT_H
//#define B2_WELD_JOINT_H

package Box2D.Dynamics.Joints
{
	//#include <Box2D/Dynamics/Joints/b2Joint.h>

	import Box2D.Common.b2Settings;
	import Box2D.Common.b2Math;
	import Box2D.Common.b2Vec2;
	import Box2D.Common.b2Vec3;
	import Box2D.Common.b2Transform;
	import Box2D.Common.b2Mat22;
	import Box2D.Common.b2Mat33;
	import Box2D.Dynamics.b2TimeStep;
	import Box2D.Dynamics.b2Body;

	/// Weld joint definition. You need to specify local anchor points
	/// where they are attached and the relative body angle. The position
	/// of the anchor points is important for computing the reaction torque.
	//struct b2WeldJointDef : public b2JointDef
	//{
		//@see b2WeldJointDef.as
	//};

	/// A weld joint essentially glues two bodies together. A weld joint may
	/// distort somewhat because the island constraint solver is approximate.
	public class b2WeldJoint extends b2Joint
	{
		include "b2WeldJoint.cpp";

	//public:

		//b2Vec2 GetAnchorA() const;
		//b2Vec2 GetAnchorB() const;

		//b2Vec2 GetReactionForce(float32 inv_dt) const;
		//float32 GetReactionTorque(float32 inv_dt) const;

	//protected:

		//friend class b2Joint;

		//b2WeldJoint(const b2WeldJointDef* def);

		//void InitVelocityConstraints(const b2TimeStep& step);
		//void SolveVelocityConstraints(const b2TimeStep& step);

		//bool SolvePositionConstraints(float32 baumgarte);

		public var m_localAnchorA:b2Vec2 = new b2Vec2 ();
		public var m_localAnchorB:b2Vec2 = new b2Vec2 ();
		public var m_referenceAngle:Number;

		public var m_impulse:b2Vec3 = new b2Vec3 ();

		public var m_mass:b2Mat33 = new b2Mat33 ();

//***********************************************************************
// hackings
//***********************************************************************

		// call by b2Body
		override public function OnBodyLocalCenterChanged (dx:Number, dy:Number, jointEdge:b2JointEdge):void
		{
			if (jointEdge == m_edgeA)
			{
				m_localAnchorA.x += dx;
				m_localAnchorA.y += dy;
			}
			else if (jointEdge == m_edgeB)
			{
				m_localAnchorB.x += dx;
				m_localAnchorB.y += dy;
			}
		}
		
      override public function NotifyAnchorPositionChanged (newWorldX:Number, newWorldY:Number, isAnchorA:Boolean):void
      {
         worldAnchor.x = newWorldX; worldAnchor.y = newWorldY;
         
         if (isAnchorA)
         {
            m_bodyA.GetLocalPoint_Output (worldAnchor, m_localAnchorA);
         }
         else
         {
            m_bodyB.GetLocalPoint_Output (worldAnchor, m_localAnchorB);
         }
      }		

      private static var worldAnchor:b2Vec2 = new b2Vec2 ();

      override protected function NotifyBodyChanged (oldBody:b2Body, isBodyA:Boolean):void
      {
         if (isBodyA)
         {
            oldBody.GetWorldPoint_Output (m_localAnchorA, worldAnchor);
            m_bodyA.GetLocalPoint_Output (worldAnchor, m_localAnchorA);
            m_referenceAngle -= m_bodyA.GetAngle () - oldBody.GetAngle ();
         }
         else
         {
            oldBody.GetWorldPoint_Output (m_localAnchorB, worldAnchor);
            m_bodyB.GetLocalPoint_Output (worldAnchor, m_localAnchorB);
            m_referenceAngle += m_bodyB.GetAngle () - oldBody.GetAngle ();
         }
      }
      
      override public function CopyRuntimeInfosFrom (fromJoint:b2Joint):void
      {
         var fromWeldJoint:b2WeldJoint = fromJoint as b2WeldJoint;
         
         m_referenceAngle = fromWeldJoint.m_referenceAngle - (fromWeldJoint.m_bodyB.GetAngle() - fromWeldJoint.m_bodyA.GetAngle());
         m_impulse.CopyFrom (fromWeldJoint.m_impulse);
      }
	}
}

//#endif
