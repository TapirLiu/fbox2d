/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
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

//#include <Box2D/Dynamics/Contacts/b2PolygonAndCircleContact.h>
//#include <Box2D/Common/b2BlockAllocator.h>
//#include <Box2D/Dynamics/b2Fixture.h>

//#include <new>

public static function Create(fixtureA:b2Fixture, indexA:int, fixtureB:b2Fixture, indexB:int, allocator:b2BlockAllocator = null):b2Contact
{
	//void* mem = allocator->Allocate(sizeof(b2PolygonAndCircleContact));
	//return new (mem) b2PolygonAndCircleContact(fixtureA, fixtureB);
	
	return new b2PolygonAndCircleContact(fixtureA, fixtureB);
}

public static function Destroy(contact:b2Contact, allocator:b2BlockAllocator = null):void
{
	(contact as b2PolygonAndCircleContact)._b2PolygonAndCircleContact ();
	//((b2PolygonAndCircleContact*)contact)->~b2PolygonAndCircleContact();
	//allocator->Free(contact, sizeof(b2PolygonAndCircleContact));
}

public function b2PolygonAndCircleContact(fixtureA:b2Fixture, fixtureB:b2Fixture)
	//: b2Contact(fixtureA, fixtureB)
{
	super (fixtureA, 0, fixtureB, 0);
	//b2Assert(m_fixtureA->GetType() == b2Shape::e_polygon);
	//b2Assert(m_fixtureB->GetType() == b2Shape::e_circle);
}

override public function Evaluate(manifold:b2Manifold, xfA:b2Transform, xfB:b2Transform):void
{
	b2Collision.b2CollidePolygonAndCircle(	manifold,
								(m_fixtureA.GetShape() as b2PolygonShape), xfA,
								(m_fixtureB.GetShape() as b2CircleShape), xfB);
}
