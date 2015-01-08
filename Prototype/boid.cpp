#include "vector.h"
#include "boidCPU.h"

#define MAX_NEIGHBOURS 	8	// The maximum number of neighbouring locations
#define MAX_VELOCITY	10
#define MAX_FORCE		1

#define AREA_WIDTH		720
#define AREA_HEIGHT		720

class Boid {

	//TODO: Should these be here?
	Vector position;
	Vector velocity;
	Vector acceleration;

	Boid neighbouringBoids[MAX_NEIGHBOURS];
	uint8 neighbouringBoidsCount;

	Boid(int _boidID, Vector initPosition, Vector initVelocity) {

		int boidID = _boidID;

		position = initPosition;
		velocity = initVelocity;

		this->neighbouringBoids = {0};
		this->neighbouringBoidsCount = 0;
	}

	void Update(void) {
		if(this->neighbouringBoidsCount > 0) {
			acceleration.add(Separate());
			acceleration.add(Align());
			acceleration.add(Cohesion());
		}

		velocity.add(acceleration);
		velocity.limit(MAX_VELOCITY);
		position.add(velocity);
		acceleration.mul(0);

		Contain();
	}

	Vector Align(void) {
		Vector total;

		for (Boid b : this->neighbouringBoids) {
			total += b.getVelocity();
		}

		total.div(this->neighbouringBoidsCount);
		total.setMag(MAX_VELOCITY);

		Vector steer = Vector.sub(total, velocity);
		steer.limit(MAX_FORCE);

		return steer;
	}

	Vector Separate(void) {
		Vector total;
		Vector diff;

		for (Boid b : this->neighbouringBoids) {
			diff = Vector.sub(position, b.getPosition());
			diff.normalise();
			total.add(diff);
		}

		total.div(this->neighbouringBoidsCount);
		total.setMag(MAX_VELOCITY);

		Vector steer = Vector.sub(total, velocity);
		steer.limit(MAX_FORCE);

		return steer;
	}

	Vector Cohesion(void) {
		Vector total;

		for(Boid b : this->neighbouringBoids) {
			total.add(b.getPosition());
		}

		total.div(this->neighbouringBoidsCount);
		Vector steer = Seek(total);
		return steer;
	}

	Vector Seek(Vector target) {
		Vector desired = Vector.sub(target, position);
		desired.setMag(MAX_VELOCITY);

		Vector steer = Vector.sub(desired, velocity);
		steer.limit(MAX_FORCE);

		return steer;
	}

	void Contain() {
		if(position.x > AREA_WIDTH) {
			position.x = 0;
		} else if(position.x < 0) {
			position.x = AREA_WIDTH;
		}

		if(position.y > AREA_HEIGHT) {
			position.y = 0;
		} else if(position.y < 0) {
			position.y = AREA_HEIGHT;
		}
	}

	Vector getVelocity(void) {
		return velocity;
	}

	Vector getPosition(void) {
		return position;
	}
};
