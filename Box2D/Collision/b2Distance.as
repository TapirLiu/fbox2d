
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

//#ifndef B2_DISTANCE_H
//#define B2_DISTANCE_H

package Box2D.Collision
{

	//#include <Box2D/Common/b2Math.h>
	//#include <climits>

	import Box2D.Common.b2Settings;
	import Box2D.Common.b2Math;
	import Box2D.Common.b2Transform;
	import Box2D.Common.b2Vec2;

	//class b2Shape;

	/// A distance proxy is used by the GJK algorithm.
	/// It encapsulates any shape.
	//struct b2DistanceProxy
	//{
		//@see b2DistanceProxy.as
	//};

	/// Used to warm start b2Distance.
	/// Set count to zero on first call.
	//struct b2SimplexCache
	//{
		//@see b2SimplexCache.as
	//};

	/// Input for b2Distance.
	/// You have to option to use the shape radii
	/// in the computation. Even
	//struct b2DistanceInput
	//{
		//@see b2DistanceInput.as
	//};

	/// Output for b2Distance.
	//struct b2DistanceOutput
	//{
		//@see b2DistanceOutput.as
	//};

	public class b2Distance
	{
		include "b2Distance.cpp";

		/// Compute the closest points between two shapes. Supports any combination of:
		/// b2CircleShape, b2PolygonShape, b2EdgeShape. The simplex cache is input/output.
		/// On the first call set b2SimplexCache.count to zero.
		//void b2Distance(b2DistanceOutput* output,
		//				b2SimplexCache* cache,
		//				const b2DistanceInput* input);


		//////////////////////////////////////////////////////////////////////////

	} // class
} // package
//#endif
