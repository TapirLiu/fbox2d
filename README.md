*fbox2d* is a flash actionscript3 port for box2d c++ physics lib (https://github.com/erincatto/box2d).

License: zlib.

Besides fbox2d, there are other 2 actionscript3 ports of box2d: one is Box2DFlashAS3 (http://box2dflash.sourceforge.net/), the other is wck (Box2d Flash World Construction Kit, http://wiki.github.com/jesses/wck/).

The intention of fbox2d project is not to compete with the other projects. I created this project mainly for two reasons:
 * my game projects need the wonderful new features in box2d v2.10;
 * I didn't find an actionscript3 port for box2d v2.10 before I created the fbox2d project.

One principle of the fbox2d project is to make the class names and function interfaces as similar with the c++ version as possible. Here I list the differences between fbox2d and box2d c++ version:
 * because actionscript3 doesn't support polymorphism, many functions are renamed. Some constructors are changed to static functions;
 * to optimize, many local variables are moved outside of function bodies and changed to static. This means the fbox2d doesn't support multi-threads. Currently, this is not bad, for flash player also doesn't support multi-threads.

Because my game projects don't use all the features of box2d, the fbox2d has not been fully tested. For the tested part, it is quite stable now. I think it can be used in most game productions now. Here is the list of the features my game projects haven't used:
 * friction joint, gear joint, line joint, mouse joint, pulley joint, rope joint;
 * kinematics body.

*NOTICES:*
 * there are still rare overlapping even penetration and tunnelling cases. I haven't made a further investigation yet.
