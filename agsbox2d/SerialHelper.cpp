/*
 * Copyright (C) 2019  Érico Vieira Porto
 *
 * This program is free software. You can use and redistribute it
 *  under the terms and conditions of the zlib-license (see LICENCE).
 *
 * SPDX-License-Identifier: Zlib
 */

#include "SerialHelper.h"

namespace SerialHelper {

	char* IntToChar(int32 i, char * buf, char* end) {
		assert(buf + sizeof(int32) < end);

		*((int32*)buf) = i;

		return buf + sizeof(int32);
	}

	char* FloatToChar(float32 f, char * buf, char* end) {
		assert(buf + sizeof(float32) < end);

		*((float*)buf) = f;

		return buf + sizeof(float32);
	}

	char* b2Vec2ToChar(b2Vec2 vec, char * buf, char* end) {
		assert(buf + 2 * sizeof(float32) < end);

		buf = FloatToChar(vec.x, buf, end);
		return FloatToChar(vec.y, buf, end);
	}


	char* BoolToChar(bool b, char* buf, char* end) {
		assert(buf + sizeof(char) < end);

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
			printf("serialized circle\n");
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

			for (int32 i = 0; i < vertexCount; ++i) {
				buf = b2Vec2ToChar(poly->m_normals[i], buf, end);
			}

			printf("serialized rectangle\n");
		}

		return buf;
	}

	char* b2FixtureToChar(b2Fixture* b2fixture, char* buf, char* end) {
		assert(buf + 3 * sizeof(float32) + sizeof(char) < end);

		buf = FloatToChar(b2fixture->GetRestitution(), buf, end);
		buf = FloatToChar(b2fixture->GetFriction(), buf, end);
		buf = FloatToChar(b2fixture->GetDensity(), buf, end);
		buf = BoolToChar(b2fixture->IsSensor(), buf, end);

		b2Shape* shape = b2fixture->GetShape();

		buf = b2ShapeToChar(shape, buf, end);

		return buf;
	}

	char* b2BodyToChar(b2Body* b2body, char* buf, char* end) {
		printf("b2BodyToChar \n");
		assert(buf + 12 * sizeof(float32) + sizeof(int32) + 4 * sizeof(char) < end);

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
			printf("fixture count %d\n", fixturecount);
		}

		buf = IntToChar(fixturecount, buf, end);

		for (b2Fixture* fixture = b2body->GetFixtureList(); fixture; fixture = fixture->GetNext()) {
			buf = b2FixtureToChar(fixture, buf, end);
		}

		return buf;
	}


    char* b2JointToChar(b2Joint* b2joint, char* buf, char* end) {
        bool collide_connected = b2joint->GetCollideConnected();
        int joint_type = b2joint->GetType();

        buf = IntToChar( joint_type, buf, end);
	    buf = BoolToChar(collide_connected, buf, end);

        b2Body* bodyA = b2joint->GetBodyA();
        b2Body* bodyB = b2joint->GetBodyB();

        if(b2joint->GetType() == b2JointType::e_revoluteJoint) {
            b2RevoluteJoint* revoluteJoint = dynamic_cast<b2RevoluteJoint*>(b2joint);

            buf = b2Vec2ToChar(bodyA->GetLocalPoint(revoluteJoint->GetAnchorA()), buf, end);
            buf = b2Vec2ToChar(bodyB->GetLocalPoint(revoluteJoint->GetAnchorB()), buf, end);
            buf = FloatToChar(bodyB->GetAngle() - bodyA->GetAngle() - revoluteJoint->GetJointAngle(), buf, end);
            buf = FloatToChar(revoluteJoint->GetJointSpeed(), buf, end);
            buf = BoolToChar(revoluteJoint->IsLimitEnabled(), buf, end);
            buf = FloatToChar(revoluteJoint->GetLowerLimit(), buf, end);
            buf = FloatToChar(revoluteJoint->GetUpperLimit(), buf, end);
            buf = BoolToChar(revoluteJoint->IsMotorEnabled(), buf, end);
            buf = FloatToChar(revoluteJoint->GetMotorSpeed(), buf, end);
            buf = FloatToChar(revoluteJoint->GetMaxMotorTorque(), buf, end);

        } else if(b2joint->GetType() == b2JointType::e_distanceJoint) {
            b2DistanceJoint* distanceJoint = dynamic_cast<b2DistanceJoint*>(b2joint);

            buf = b2Vec2ToChar(bodyA->GetLocalPoint(distanceJoint->GetAnchorA()), buf, end);
            buf = b2Vec2ToChar(bodyB->GetLocalPoint(distanceJoint->GetAnchorB()), buf, end);
            buf = FloatToChar(distanceJoint->GetLength(), buf, end);
            buf = FloatToChar(distanceJoint->GetFrequency(), buf, end);
            buf = FloatToChar(distanceJoint->GetDampingRatio(), buf, end);

        } else if(b2joint->GetType() == b2JointType::e_pulleyJoint) {
            b2PulleyJoint* pulleyJoint = dynamic_cast<b2PulleyJoint*>(b2joint);

            buf = b2Vec2ToChar(pulleyJoint->GetGroundAnchorA(), buf, end);
            buf = b2Vec2ToChar(pulleyJoint->GetGroundAnchorB(), buf, end);
            buf = b2Vec2ToChar(bodyA->GetLocalPoint(pulleyJoint->GetAnchorA()), buf, end);
            buf = b2Vec2ToChar(bodyB->GetLocalPoint(pulleyJoint->GetAnchorB()), buf, end);
            buf = FloatToChar((pulleyJoint->GetGroundAnchorA() - pulleyJoint->GetAnchorA()).Length(), buf, end);
            buf = FloatToChar((pulleyJoint->GetGroundAnchorB() - pulleyJoint->GetAnchorB()).Length(), buf, end);
            buf = FloatToChar(pulleyJoint->GetRatio(), buf, end);

        } else if(b2joint->GetType() == b2JointType::e_mouseJoint) {
            b2MouseJoint* mouseJoint = dynamic_cast<b2MouseJoint*>(b2joint);

            buf = b2Vec2ToChar(mouseJoint->GetTarget(), buf, end);
            buf = b2Vec2ToChar(mouseJoint->GetAnchorB(), buf, end);
            buf = FloatToChar(mouseJoint->GetMaxForce(), buf, end);
            buf = FloatToChar(mouseJoint->GetFrequency(), buf, end);
            buf = FloatToChar(mouseJoint->GetDampingRatio(), buf, end);

        } else if(b2joint->GetType() == b2JointType::e_motorJoint) {
            b2MotorJoint* motorJoint = dynamic_cast<b2MotorJoint*>(b2joint);

            buf = b2Vec2ToChar(motorJoint->GetLinearOffset(), buf, end);
            buf = FloatToChar(motorJoint->GetAngularOffset(), buf, end);
            buf = FloatToChar(motorJoint->GetMaxForce(), buf, end);
            buf = FloatToChar(motorJoint->GetMaxTorque(), buf, end);
            buf = FloatToChar(motorJoint->GetCorrectionFactor(), buf, end);
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

	char* CharTob2Shape(b2Shape** pb2shape, char * buf) {
		int32 itype;
		b2Shape::Type type;

		buf = CharToInt(itype, buf);
		type = (b2Shape::Type)itype;

		if (type == b2Shape::e_circle)
		{
			(*pb2shape) = new b2CircleShape();
			float32 radius;
			b2Vec2 center;

			buf = CharToFloat(radius, buf);
			buf = CharTob2Vec2(center, buf);

			((b2CircleShape*)(*pb2shape))->m_radius = radius;
			((b2CircleShape*)(*pb2shape))->m_p = center;


			printf(" deserialized circle\n");
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

			printf("vertexCount %d ]\n", vertexCount);

			(*pb2shape) = new b2PolygonShape();

			((b2PolygonShape*)(*pb2shape))->m_count = vertexCount;

			if (vertexCount > 2) {
				for (int i = 0; i < vertexCount; i++)
					buf = CharTob2Vec2(((b2PolygonShape*)(*pb2shape))->m_vertices[i], buf);

				for (int i = 0; i < vertexCount; i++)
					buf = CharTob2Vec2(((b2PolygonShape*)(*pb2shape))->m_normals[i], buf);
			}

			((b2PolygonShape*)(*pb2shape))->m_centroid.SetZero();

			printf(" deserialized rectangle\n");
		}

		return buf;
	}

	char* CharTob2FixtureDef(b2FixtureDef* b2fixturedef, char* buf) {
		buf = CharToFloat(b2fixturedef->restitution, buf);
		buf = CharToFloat(b2fixturedef->friction, buf);
		buf = CharToFloat(b2fixturedef->density, buf);
		buf = CharToBool(b2fixturedef->isSensor, buf);

		b2Shape* shape = nullptr;
		buf = CharTob2Shape(&shape, buf);

		b2fixturedef->shape = shape;

		printf("Restitution: %f ; Friction: %f ; Density: %f ; IsSensor %d\n",
						b2fixturedef->restitution, b2fixturedef->friction, b2fixturedef->density, b2fixturedef->isSensor);

		if(shape->GetType() == b2Shape::e_circle){
			printf(" got e_circle\n");
		} else if(shape->GetType() == b2Shape::e_polygon){
			printf(" got e_polygon\n");
		}

		return buf;
	}

	char* CharTob2Body(b2BodyDef &b2bodydef, b2Body** pb2body, b2World* world, char* buf) {
		printf("deserialized body\n");

		b2Vec2 position;
		float32 angle;
		int32 bodytype;
		float32 linearDamping;
		float32 angularDamping;
		b2Vec2 linearVelocity;
		float32 angularVelocity;
		bool fixedRotation;
		bool bullet;
		bool active;
		bool awake;

		b2MassData massData;

		buf = CharTob2Vec2(position, buf);
		buf = CharToFloat(angle, buf);
		buf = CharToInt(bodytype, buf);
		buf = CharToFloat(linearDamping, buf);
		buf = CharToFloat(angularDamping, buf);
		buf = CharTob2Vec2(linearVelocity, buf);
		buf = CharToFloat(angularVelocity, buf);
		buf = CharToBool(fixedRotation, buf);
		buf = CharToBool(bullet, buf);
		buf = CharToBool(active, buf);
		buf = CharToBool(awake, buf);
		b2bodydef.type = (b2BodyType) bodytype;

		b2bodydef.position.Set(position.x, position.y);
		b2bodydef.angle = angle;
		b2bodydef.fixedRotation = fixedRotation;

		if (b2bodydef.type != b2BodyType::b2_staticBody) {
			b2bodydef.linearDamping = linearDamping;
			b2bodydef.angularDamping = angularDamping;
			b2bodydef.linearVelocity.Set(linearVelocity.x, linearVelocity.y);
			b2bodydef.angularVelocity = angularVelocity;
			b2bodydef.bullet = bullet;
			b2bodydef.active = active;
			b2bodydef.awake = awake;
		}

		buf = CharToFloat(massData.mass, buf);
		buf = CharTob2Vec2(massData.center, buf);
		buf = CharToFloat(massData.I, buf);

		(*pb2body) = world->CreateBody(&b2bodydef);

		int fixturecount;
		buf = CharToInt(fixturecount, buf);

		for (int i = 0; i < fixturecount; i++) {
			b2FixtureDef* fixturedef = new b2FixtureDef;
			buf = CharTob2FixtureDef(fixturedef, buf);
			(*pb2body)->CreateFixture(fixturedef);
		}

		if(b2bodydef.type != b2BodyType::b2_staticBody &&
			(massData.mass != 0.0f ||
			massData.center.x != 0.0f ||
			massData.center.y != 0.0f ||
		    massData.I != 0.0)) {
				printf("set mass data");
				(*pb2body)->SetMassData(&massData);
	  }

		return buf;
	}

    char* CharTob2JointDef(b2JointDef *b2jointdef, char* buf) {
        int32 jointType;
        bool collide_connected;

        buf = CharToInt(jointType, buf);
        buf = CharToBool(collide_connected, buf);

        if(jointType == b2JointType::e_revoluteJoint) {
            b2RevoluteJointDef* revoluteJointDef = new b2RevoluteJointDef;
            b2jointdef = revoluteJointDef;

        } else if(jointType == b2JointType::e_distanceJoint) {
            b2DistanceJointDef* distanceJointDef = new b2DistanceJointDef;
            b2jointdef = distanceJointDef;

            b2Vec2 localAnchorA;
            b2Vec2 localAnchorB;
            float32 length;
            float32 frequency;
            float32 dampingRatio;

            buf = CharTob2Vec2(localAnchorA, buf);
            buf = CharTob2Vec2(localAnchorB, buf);
            buf = CharToFloat( length ,buf);
            buf = CharToFloat( frequency ,buf);
            buf = CharToFloat( dampingRatio ,buf);

            distanceJointDef->localAnchorA = localAnchorA;
            distanceJointDef->localAnchorB = localAnchorB;
            distanceJointDef->length = length;
            distanceJointDef->frequencyHz = frequency;
            distanceJointDef->dampingRatio = dampingRatio;

        } else if(jointType == b2JointType::e_pulleyJoint) {
            b2PulleyJointDef* pulleyJointDef = new b2PulleyJointDef;
            b2jointdef = pulleyJointDef;

            b2Vec2 groundAnchorA;
            b2Vec2 groundAnchorB;
            b2Vec2 localAnchorA;
            b2Vec2 localAnchorB;
            float32 lengthA;
            float32 lengthB;
            float32 ratio;

            buf = CharTob2Vec2(groundAnchorA, buf);
            buf = CharTob2Vec2(groundAnchorB, buf);
            buf = CharTob2Vec2(localAnchorA, buf);
            buf = CharTob2Vec2(localAnchorB, buf);
            buf = CharToFloat( lengthA ,buf);
            buf = CharToFloat( lengthB ,buf);
            buf = CharToFloat( ratio ,buf);

            pulleyJointDef->groundAnchorA = groundAnchorA;
            pulleyJointDef->groundAnchorB = groundAnchorB;
            pulleyJointDef->localAnchorA = localAnchorA;
            pulleyJointDef->localAnchorB = localAnchorB;
            pulleyJointDef->lengthA = lengthA;
            pulleyJointDef->lengthB = lengthB;
            pulleyJointDef->ratio = ratio;

        } else if(jointType == b2JointType::e_mouseJoint) {
            b2MouseJointDef* mouseJointDef = new b2MouseJointDef;
            b2jointdef = mouseJointDef;

            b2Vec2 target;
            float32 maxForce;
            float32 frequency;
            float32 dampingRatio;

            buf = CharTob2Vec2(target, buf);
            buf = CharToFloat( maxForce ,buf);
            buf = CharToFloat( frequency ,buf);
            buf = CharToFloat( dampingRatio ,buf);

            mouseJointDef->target = target;
            mouseJointDef->maxForce = maxForce;
            mouseJointDef->frequencyHz = frequency;
            mouseJointDef->dampingRatio = dampingRatio;

        } else if(jointType == b2JointType::e_motorJoint) {
            b2MotorJointDef* motorJointDef = new b2MotorJointDef;
            b2jointdef = motorJointDef;

            b2Vec2 linearOffset;
            float32 angularOffset;
            float32 maxForce;
            float32 maxTorque;
            float32 correctionFactor;

            buf = CharTob2Vec2(linearOffset, buf);
            buf = CharToFloat( angularOffset ,buf);
            buf = CharToFloat( maxForce ,buf);
            buf = CharToFloat( maxTorque ,buf);
            buf = CharToFloat( correctionFactor ,buf);

            motorJointDef->linearOffset = linearOffset;
            motorJointDef->angularOffset = angularOffset;
            motorJointDef->maxForce = maxForce;
            motorJointDef->maxTorque = maxTorque;
            motorJointDef->correctionFactor = correctionFactor;
        }

        if(b2jointdef) {
            b2jointdef->collideConnected = collide_connected;
        }


        return buf;
	}
}
