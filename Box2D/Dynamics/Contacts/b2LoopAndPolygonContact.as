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

//#ifndef B2_LOOP_AND_POLYGON_CONTACT_H
//#define B2_LOOP_AND_POLYGON_CONTACT_H

package Box2D.Dynamics.Contacts
{
	//#include <Box2D/Dynamics/Contacts/b2Contact.h>

	import Box2D.Common.b2BlockAllocator;
	import Box2D.Common.b2Transform;
	import Box2D.Dynamics.b2Body;
	import Box2D.Dynamics.b2Fixture;
	import Box2D.Collision.b2Collision;
	import Box2D.Collision.b2Manifold;
	import Box2D.Collision.Shapes.b2PolygonShape;
	import Box2D.Collision.Shapes.b2EdgeShape;
	import Box2D.Collision.Shapes.b2LoopShape;

	//class b2BlockAllocator;

	public class b2LoopAndPolygonContact extends b2Contact
	{
		include "b2LoopAndPolygonContact.cpp";

	//public:
		//static b2Contact* Create(	b2Fixture* fixtureA, int32 indexA,
		//							b2Fixture* fixtureB, int32 indexB, b2BlockAllocator* allocator);
		//static void Destroy(b2Contact* contact, b2BlockAllocator* allocator);

		//b2LoopAndPolygonContact(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB);
		//~b2LoopAndPolygonContact() {}
		public function _b2LoopAndPolygonContact():void {}

		//void Evaluate(b2Manifold* manifold, const b2Transform& xfA, const b2Transform& xfB);
	}
}
//#endif
