#include "SerialHelper.h"

namespace SerialHelper {

	char* IntToChar(int32 i, char * buf, char* end) {
		assert(buf + sizeof(int32) > end);

		*((int32*)buf) = i;

		return buf + sizeof(int32);
	}

	char* FloatToChar(float32 f, char * buf, char* end) {
		assert(buf + sizeof(float32) > end);

		*((float*)buf) = f;

		return buf + sizeof(float32);
	}

	char* b2Vec2ToChar(b2Vec2 vec, char * buf, char* end) {
		assert(buf + 2 * sizeof(float32) > end);

		buf = FloatToChar(vec.x, buf, end);
		return FloatToChar(vec.y, buf, end);
	}


	char* BoolToChar(bool b, char* buf, char* end) {
		assert(buf + sizeof(char) > end);

		if (b) {
			*buf = 1;
		}
		else {
			*buf = 0;
		}
		return buf + sizeof(char);
	}

	char* b2ShapeToChar(b2Shape* b2shape, char* buf, char* end) {
		buf = IntToChar((int32)b2shape->GetType(), buf, end);

		if (b2shape->GetType() == b2Shape::e_circle)
		{
			b2CircleShape* circle = (b2CircleShape*)b2shape;
			buf = FloatToChar(circle->m_radius, buf, end);
			buf = b2Vec2ToChar(circle->m_p, buf, end); //circle center

		}
		else if (b2shape->GetType() == b2Shape::e_edge)
		{
			//haven't implemented yet
		}
		else if (b2shape->GetType() == b2Shape::e_chain)
		{
			//haven't implemented yet
		}
		else if (b2shape->GetType() == b2Shape::e_polygon)
		{
			b2PolygonShape* poly = (b2PolygonShape*)b2shape;
			int32 vertexCount = poly->m_count;
			b2Assert(vertexCount <= b2_maxPolygonVertices);

			buf = IntToChar(vertexCount, buf, end);
			for (int32 i = 0; i < vertexCount; ++i) {
				buf = b2Vec2ToChar(poly->m_vertices[i], buf, end);
			}
		}

		return buf;
	}

	char* b2FixtureToChar(b2Fixture* b2fixture, char* buf, char* end) {
		assert(buf + 3 * sizeof(float32) + sizeof(char) > end);

		buf = FloatToChar(b2fixture->GetRestitution(), buf, end);
		buf = FloatToChar(b2fixture->GetFriction(), buf, end);
		buf = FloatToChar(b2fixture->GetDensity(), buf, end);
		buf = BoolToChar(b2fixture->IsSensor(), buf, end);

		b2Shape* shape = b2fixture->GetShape();

		buf = b2ShapeToChar(shape, buf, end);

		return buf;
	}

	char* b2BodyToChar(b2Body* b2body, char* buf, char* end) {
		assert(buf + 12 * sizeof(float32) + sizeof(int32) + 4 * sizeof(char) > end);

		buf = b2Vec2ToChar(b2body->GetPosition(), buf, end);
		buf = FloatToChar(b2body->GetAngle(), buf, end);
		buf = IntToChar((int32)b2body->GetType(), buf, end);
		buf = FloatToChar(b2body->GetLinearDamping(), buf, end);
		buf = FloatToChar(b2body->GetAngularDamping(), buf, end);
		buf = b2Vec2ToChar(b2body->GetLinearVelocity(), buf, end);
		buf = FloatToChar(b2body->GetAngularVelocity(), buf, end);
		buf = BoolToChar(b2body->IsFixedRotation(), buf, end);
		buf = BoolToChar(b2body->IsBullet(), buf, end);
		buf = BoolToChar(b2body->IsActive(), buf, end);
		buf = BoolToChar(b2body->IsAwake(), buf, end);

		b2MassData massData;
		b2body->GetMassData(&massData);

		buf = FloatToChar(massData.mass, buf, end);
		buf = b2Vec2ToChar(massData.center, buf, end);
		buf = FloatToChar(massData.I, buf, end);

		int fixturecount = 0;
		for (b2Fixture* fixture = b2body->GetFixtureList(); fixture; fixture = fixture->GetNext()) {
			fixturecount++;
		}

		buf = IntToChar(fixturecount, buf, end);

		for (b2Fixture* fixture = b2body->GetFixtureList(); fixture; fixture = fixture->GetNext()) {
			buf = b2FixtureToChar(fixture, buf, end);
		}

		return buf;
	}

	char* CharToBool(bool &b, char* buf) {
		if (*buf == 0) {
			b = false;
		}
		else {
			b = true;
		}
		return buf + sizeof(char);
	}

	char* CharToInt(int32 &i, char * buf) {
		i = *((int32*)buf);
		return buf + sizeof(int32);
	}

	char* CharToFloat(float32 &f, char * buf) {
		f = *((float32*)buf);
		return buf + sizeof(float32);
	}

	char* CharTob2Vec2(b2Vec2 &vec, char * buf) {
		buf = CharToFloat(vec.x, buf);
		return CharToFloat(vec.y, buf);
	}

	char* CharTob2Shape(b2Shape* b2shape, char * buf) {
		int32 itype;
		b2BodyType type;

		if (b2shape != nullptr) {
			delete b2shape;
		}

		buf = CharToInt(itype, buf);
		type = (b2BodyType)itype;

		if (type == b2Shape::e_circle)
		{
			b2shape = new b2CircleShape;
			float32 radius;
			b2Vec2 center;

			buf = CharToFloat(radius, buf);
			buf = CharTob2Vec2(center, buf);

			((b2CircleShape*)b2shape)->m_radius = radius;
			((b2CircleShape*)b2shape)->m_p = center;
		}
		else if (type == b2Shape::e_edge)
		{
			//haven't implemented yet
		}
		else if (type == b2Shape::e_chain)
		{
			//haven't implemented yet
		}
		else if (type == b2Shape::e_polygon)
		{
			int vertexCount;
			buf = CharToInt(vertexCount, buf);
			assert(vertexCount <= b2_maxPolygonVertices);

			b2shape = new b2PolygonShape;

			if (vertexCount > 2) {
				for (int i = 0; i < vertexCount; i++)
					buf = CharTob2Vec2(((b2PolygonShape*)b2shape)->m_vertices[i], buf);
			}
		}

		return buf;
	}

	char* CharTob2FixtureDef(b2FixtureDef* b2fixturedef, char* buf) {
		buf = CharToFloat(b2fixturedef->restitution, buf);
		buf = CharToFloat(b2fixturedef->friction, buf);
		buf = CharToFloat(b2fixturedef->density, buf);
		buf = CharToBool(b2fixturedef->isSensor, buf);

		b2Shape* shape = nullptr;
		buf = CharTob2Shape(shape, buf);

		b2fixturedef->shape = shape;

		return buf;
	}

	char* CharTob2Body(b2Body* b2body, char* buf) {
		b2Vec2 position;
		float32 angle;
		int32 bodytype;
		float32 linear_damping;
		float32 angular_damping;
		b2Vec2 linear_velocity;
		float32 angular_velocity;
		bool is_fixed_rotation;
		bool is_bullet;
		bool is_active;
		bool is_awake;

		b2MassData massData;

		buf = CharTob2Vec2(position, buf);
		buf = CharToFloat(angle, buf);
		buf = CharToInt(bodytype, buf);
		buf = CharToFloat(linear_damping, buf);
		buf = CharToFloat(angular_damping, buf);
		buf = CharTob2Vec2(linear_velocity, buf);
		buf = CharToFloat(angular_velocity, buf);
		buf = CharToBool(is_fixed_rotation, buf);
		buf = CharToBool(is_bullet, buf);
		buf = CharToBool(is_active, buf);
		buf = CharToBool(is_awake, buf);

		buf = CharToFloat(massData.mass, buf);
		buf = CharTob2Vec2(massData.center, buf);
		buf = CharToFloat(massData.I, buf);

		b2body->SetTransform(position, angle);
		b2body->SetType((b2BodyType)bodytype);
		b2body->SetLinearDamping(linear_damping);
		b2body->SetAngularDamping(angular_damping);
		b2body->SetLinearVelocity(linear_velocity);
		b2body->SetAngularVelocity(angular_velocity);
		b2body->SetFixedRotation(is_fixed_rotation);
		b2body->SetBullet(is_bullet);
		b2body->SetActive(is_active);
		b2body->SetAwake(is_awake);

		int fixturecount;
		buf = CharToInt(fixturecount, buf);

		for (int i = 0; i < fixturecount; i++) {
			b2FixtureDef* fixturedef = new b2FixtureDef;
			buf = CharTob2FixtureDef(fixturedef, buf);
			b2body->CreateFixture(fixturedef);
		}
		b2body->SetMassData(&massData);

		return buf;
	}
}