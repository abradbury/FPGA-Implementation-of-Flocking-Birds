class Vector {
	public:
		int8 x;
		int8 y;
		int8 z;

		Vector();
		Vector(int8 x_, int8 y_, int8 z_);

		void add(Vector v);
		void sub(Vector v);
		void mul(uint8 n);
		void div(uint8 n);
		uint8 mag();
		void normalise();
		void bound(uint8 n);
		void rand(uint8 min, uint8 max);
		bool empty();

		void setMag(uint8 mag);
		void limit(uint8 max);

		static Vector add(Vector v1, Vector v2);
		static Vector sub(Vector v1, Vector v2);

		// TODO: Is this needed?
		friend std::ostream& operator<<(std::ostream& os, const Vector& v);
	private:

};
