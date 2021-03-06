/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

//#ifndef B2_DISTANCE_JOINT_H
//#define B2_DISTANCE_JOINT_H

package Box2D.Dynamics.Joints
{

	//#include <Box2D/Dynamics/Joints/b2Joint.h>
	import Box2D.Common.b2Settings;
	import Box2D.Common.b2Math;
	import Box2D.Common.b2Vec2;
	import Box2D.Dynamics.b2TimeStep;
	import Box2D.Dynamics.b2Body;


	/// Distance joint definition. This requires defining an
	/// anchor point on both bodies and the non-zero length of the
	/// distance joint. The definition uses local anchor points
	/// so that the initial configuration can violate the constraint
	/// slightly. This helps when saving and loading a game.
	/// @warning Do not use a zero or short length.
	//struct b2DistanceJointDef : public b2JointDef
	//{
		//@see b2DistanceJointDef.as
	//};

	/// A distance joint constrains two points on two bodies
	/// to remain at a fixed distance from each other. You can view
	/// this as a massless, rigid rod.
	public class b2DistanceJoint extends b2Joint
	{
		include "b2DistanceJoint.cpp";

	//public:

		//b2Vec2 GetAnchorA() const;
		//b2Vec2 GetAnchorB() const;

		/// Get the reaction force given the inverse time step.
		/// Unit is N.
		//b2Vec2 GetReactionForce(float32 inv_dt) const;

		/// Get the reaction torque given the inverse time step.
		/// Unit is N*m. This is always zero for a distance joint.
		//float32 GetReactionTorque(float32 inv_dt) const;

		/// Set/get the natural length.

		//void SetLength(float32 length);
		/// Manipulating the length can lead to non-physical behavior when the frequency is zero.
		//float32 GetLength() const;

		// Set/get frequency in Hz.
		//void SetFrequency(float32 hz);
		//float32 GetFrequency() const;

		// Set/get damping ratio.
		//void SetDampingRatio(float32 ratio);
		//float32 GetDampingRatio() const;

	//protected:

		//b2DistanceJoint(const b2DistanceJointDef* data);

		//void InitVelocityConstraints(const b2TimeStep& step);
		//void SolveVelocityConstraints(const b2TimeStep& step);
		//bool SolvePositionConstraints(float32 baumgarte);

		public var m_localAnchor1:b2Vec2 = new b2Vec2 ();
		public var m_localAnchor2:b2Vec2 = new b2Vec2 ();
		public var m_u:b2Vec2 = new b2Vec2 ();
		public var m_frequencyHz:Number;
		public var m_dampingRatio:Number;
		public var m_gamma:Number;
		public var m_bias:Number;
		public var m_impulse:Number;
		public var m_mass:Number;
		public var m_length:Number;
	//}

	// inline

		public function SetLength(length:Number):void
		{
			m_length = length;
		}

		public function GetLength():Number
		{
			return m_length;
		}

		public function SetFrequency(hz:Number):void
		{
			m_frequencyHz = hz;
		}

		public function GetFrequency():Number
		{
			return m_frequencyHz;
		}

		public function SetDampingRatio(ratio:Number):void
		{
			m_dampingRatio = ratio;
		}

		public function GetDampingRatio():Number
		{
			return m_dampingRatio;
		}

//***********************************************************************
// hackings
//***********************************************************************

      public var mSpringConstant:Number = -1.0;

		// call by b2Body
		override public function OnBodyLocalCenterChanged (dx:Number, dy:Number, jointEdge:b2JointEdge):void
		{
			if (jointEdge == m_edgeA)
			{
				m_localAnchor1.x += dx;
				m_localAnchor1.y += dy;
			}
			else if (jointEdge == m_edgeB)
			{
				m_localAnchor2.x += dx;
				m_localAnchor2.y += dy;
			}
		}
		
      override public function NotifyAnchorPositionChanged (newWorldX:Number, newWorldY:Number, isAnchorA:Boolean):void
      {
         worldAnchor.x = newWorldX; worldAnchor.y = newWorldY;
         
         if (isAnchorA)
         {
            m_bodyA.GetLocalPoint_Output (worldAnchor, m_localAnchor1);
         }
         else
         {
            m_bodyB.GetLocalPoint_Output (worldAnchor, m_localAnchor2);
         }
      }		

		private static var worldAnchor:b2Vec2 = new b2Vec2 ();

		override protected function NotifyBodyChanged (oldBody:b2Body, isBodyA:Boolean):void
		{
			if (isBodyA)
			{
				oldBody.GetWorldPoint_Output (m_localAnchor1, worldAnchor);
				m_bodyA.GetLocalPoint_Output (worldAnchor, m_localAnchor1);
			}
			else
			{
				oldBody.GetWorldPoint_Output (m_localAnchor2, worldAnchor);
				m_bodyB.GetLocalPoint_Output (worldAnchor, m_localAnchor2);
			}
		}
		
		override public function CopyRuntimeInfosFrom (fromJoint:b2Joint):void
      {
         var fromDistanceJoint:b2DistanceJoint = fromJoint as b2DistanceJoint;
         
         m_impulse = fromDistanceJoint.m_impulse;
         m_length = fromDistanceJoint.m_length;
      }
		
		override public function OnScaled (scaleRatio:Number):void
      {
          m_length *= scaleRatio;
      }

	} // class
} // package
//#endif
