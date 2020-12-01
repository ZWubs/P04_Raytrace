import 'dart:math';

import 'maths.dart';
import 'scene.dart';


class AA {

	static List<Ray> generate_rays( Camera camera, num samples, num u, num v, Size2i resolution ) {

		List<Ray> rays = [];

		Point origin = camera.frame.o;
		Point screen;

		for(var aay = 0; aay < samples; aay++) {
			for(var aax = 0; aax < samples; aax++) {

				screen = camera.frame.o
					+ camera.frame.x * ( camera.sensorSize.width  * camera.sensorDistance * ( u - 0.5 + ( 1 / samples / 2 ) / resolution.width + ( aax / samples ) / resolution.width ) )
					+ camera.frame.y * ( camera.sensorSize.height * camera.sensorDistance * ( v - 0.5 + ( 1 / samples / 2 ) / resolution.height + ( aay / samples ) / resolution.height ) )
					+ camera.frame.z * ( - camera.sensorDistance );

				rays.add( new Ray( origin, Direction.fromVector( screen - origin ) ) );

			}

		}

		return rays;

	}

}
