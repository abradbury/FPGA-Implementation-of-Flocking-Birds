#import "vector.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

class Vector {
	//TODO: Why should these be redefined here (from the header)?
	int8 x;
	int8 y;
	int8 z;

	// Constructors ////////////////////////////////////////////////////////////////
	Vector::Vector() {
		x = 0;
		y = 0;
		z = 0;
	}

	Vector::Vector(int8 x_, int8 y_, int8 z_) {
		x = x_;
		y = y_;
		z = z_;
	}

	// Basic Operations ////////////////////////////////////////////////////////////
	void Vector::add(Vector v) {
		x = x + v.x;
		y = y + v.y;
		z = z + v.z;
	}

	void Vector::sub(Vector v) {
		x = x - v.x;
		y = y - v.y;
		z = z - v.z;
	}

	void Vector::mul(uint8 n) {
		x = x * n;
		y = y * n;
		z = z * n;
	}

	void Vector::div(uint8 n) {
		if (n != 0) {
			x = x / n;
			y = y / n;
			z = z / n;
		}
	}

	// Static Operations /////////////////////////////////////////////////////////
	static Vector Vector::add(Vector v1, Vector v2) {
		Vector v3 = new Vector(v1.x + v2.x, v1.y + v2.y);
		return v3;
	}

	static Vector Vector::sub(Vector v1, Vector v2) {
		Vector v3 = new Vector(v1.x - v2.x, v1.y - v2.y);
		return v3;
	}

	// Advanced Operations /////////////////////////////////////////////////////////

	// http://stackoverflow.com/a/5009006
	void Vector::rand2D(int8 xMin, int8 xMax, int yMin, int yMax) {
		x = xMin + (rand() % (int)(xMax - xMin + 1));
		y = yMin + (rand() % (int)(yMax - yMin + 1));
	}


	uint8 Vector::mag() {
		return (uint8)round(sqrt(double(x*x + y*y + z*z)));
	}

	void Vector::setMag(uint8 mag) {
		normalise();
		mul(mag);
	}

	void Vector::normalise() {
		uint8 m = mag();
		div(m);
	}

	void Vector::limit(uint8 max) {
		if(1 > max) {
			//TODO
			normalise();
			mul(max);
		}
	}

	void Vector::bound(uint8 n) {
		// TODO: The is technically not binding the speed, which is the magnitude
		if(x > n) x = n;
		if(y > n) y = n;
		if(z > n) z = n;
	}

	void Vector::rand(uint8 min, uint8 max) {
		// TODO
	}

	bool Vector::empty() {
		bool result = true;

		if(x) result = false;
		else if(y) result = false;
		else if(z) result = false;

		return result;
	}

	// Other ///////////////////////////////////////////////////////////////////////
	std::ostream& operator <<(std::ostream& os, const Vector& v) {
		os << "[" << v.x << ", " << v.y << ", " << v.z << "]";
		return os;
	}
};

