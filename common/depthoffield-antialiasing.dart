import 'dart:math';

import 'maths.dart';
import 'scene.dart';

var RAND = Random(0);

class DOFAA {

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

	static void dof_sampler( List<Ray> rays, Camera camera, num dof_samples, Point screen ) {

		Point origin;

		for( num i = 0; i < dof_samples; i++ ) {

			if( camera.aperture == 0.0 ) origin = camera.frame.o;
			else origin = camera.frame.l2wPoint( random_in_unit_disk( camera.aperture ) );

			rays.add( new Ray( origin, Direction.fromVector( screen - origin ) ) );

		}

	}

	static List<Ray> generate_rays( Camera camera, num dof_samples, num aa_samples, Point precalculated_screen, num u, num v, Size2i resolution, num depth ) {

		List<Ray> rays = [];

		if( aa_samples <= 1 ) dof_sampler( rays, camera, dof_samples, precalculated_screen );
		else {

			Point screen = Point.fromVector( precalculated_screen );

			for(var aay = 0; aay < aa_samples; aay++) {
				for(var aax = 0; aax < aa_samples; aax++) {

					screen = camera.frame.o
						+ camera.frame.x * ( camera.sensorSize.width  * camera.sensorDistance * ( u - 0.5 + ( 1 / aa_samples / 2 ) / resolution.width + ( aax / aa_samples ) / resolution.width ) )
						+ camera.frame.y * ( camera.sensorSize.height * camera.sensorDistance * ( v - 0.5 + ( 1 / aa_samples / 2 ) / resolution.height + ( aay / aa_samples ) / resolution.height ) )
						+ camera.frame.z * ( - camera.sensorDistance );

					dof_sampler( rays, camera, dof_samples, screen );

				}
			}

		}

		return rays;

	}

}
