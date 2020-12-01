import 'dart:math';

import 'maths.dart';
import 'scene.dart';

var RAND = Random(0);

class DOF {

	static double random( num min, num max ) {
		return RAND.nextDouble() * ( max - min ) + min;
	}

	static Point random_in_unit_disk( num radius ) {
		while( true ) {
			Point p = new Point( random( -radius, radius ), random( -radius, radius ), 0.0 );
			if( p.lengthSquared >= 1.0 ) continue;
			return p;
		}
	}

	static List<Ray> generate_rays( Camera camera, num samples, Point screen ) {

		List<Ray> rays = [];

		Point origin;

		for( num i = 0; i < samples; i++ ) {

			if( camera.aperture == 0.0 ) origin = camera.frame.o;
			else origin = camera.frame.l2wPoint( random_in_unit_disk( camera.aperture ) );

			rays.add( new Ray( origin, Direction.fromVector( screen - origin ) ) );

		}

		return rays;

	}

}
