# agsbox2d

[![Build Status](https://dev.azure.com/ericoporto/agsbox2d/_apis/build/status/ericoporto.agsbox2d?branchName=master)](https://dev.azure.com/ericoporto/agsbox2d/_build/latest?definitionId=11&branchName=master)

AgsBox2D is a physics plugin for Adventure Game Studio that gives access to the Box2D library created by Erin Catto.
Because I never used Box2D directly before, I tried to make the API similar to Love physics module.

![](agsbox2d_demo.gif)

AgsBox2D is still in early development.

- [In development warning](#in-development-warning)
- [Usage example](#usage-example)
- [Script API](#script-api)
  * [AgsBox2D](#agsbox2d)
  * [Body](#body)
  * [World](#world)
  * [Shape](#shape)
  * [Fixture](#fixture)
  * [Joint](#joint)
  * [Contact](#contact)
- [Download agsbox2d](#download-agsbox2d)
- [Building agsbox2d](#building-agsbox2d)
- [License and Author](#license-and-author)

## In development warning

AgsBox2D is in development and you can reach uncommon bugs, like unstable load
from regular ags save. This plugin does support regulas AGS Save and Load, but
I can have forgotten something, so if you hit this, just [open an issue here](https://github.com/ericoporto/agsbox2d/issues/new).

Still, if you want to experiment with it and report your findings, this README
should prove itself useful and I appreciate any help in making this plugin work
well with AGS.

## Usage example

Below we will do a very simple example that creates a ground, and adds a
ball and a box. The ball is controlled by keyboard arrows.

```AGS Script
// room script file
World* world;
Overlay* ov;

struct Physical {
  Body* body;
  Shape* shape;
  Fixture* fixture;
};

Physical ground;
Physical ball;

function room_Load()
{
  if(world == null){
    AgsBox2D.SetMeter(32.0);
    world = AgsBox2D.CreateWorld(0.0, 9.8*AgsBox2D.GetMeter());
    
    ground.body = AgsBox2D.CreateBody(world, 160.0, 160.0, eBodyStatic);
    ground.shape = AgsBox2D.CreateRectangleShape(320.0, 40.0);
    ground.fixture = AgsBox2D.CreateFixture(ground.body, ground.shape);

    ball.body = AgsBox2D.CreateBody(world, 160.0, 40.0, eBodyDynamic);
    ball.shape = AgsBox2D.CreateCircleShape(20.0);
    ball.fixture = AgsBox2D.CreateFixture(ball.body, ball.shape, 1.0); 
    ball.fixture.Restitution = 0.5;
    
    AgsBox2D.CreateFixture(AgsBox2D.CreateBody(world, 80.0, 60.0, eBodyDynamic), 
                           AgsBox2D.CreateRectangleShape(30.0, 20.0), 5.0);
  }
}

function room_RepExec()
{
  if(IsKeyPressed(eKeyLeftArrow)) ball.body.ApplyForce(-500.0, 0.0);
  if(IsKeyPressed(eKeyRightArrow)) ball.body.ApplyForce(500.0, 0.0);
  if(IsKeyPressed(eKeyUpArrow) && ball.body.IsTouching(ground.body)){
    ball.body.ApplyForce(0.0, -6000.0);
    ball.body.SetLinearVelocity(0.0, 0.0);
  }
  
  if(ov!=null && ov.Valid) ov.Remove();  
  ov = Overlay.CreateGraphical(0, 0, world.GetDebugSprite(), true);
  
  world.Step(1.0/IntToFloat(GetGameSpeed()), 8, 3);
}
```

Check [agsbox2d_demo](https://github.com/ericoporto/agsbox2d/tree/master/agsbox2d_demo) AGS Game project by loading it on AGS!

---

## Script API

### AgsBox2D

#### `void AgsBox2D.SetMeter(float meter)`

Sets how many pixels equals to a meter. Default is 32 pixels per meter.

***Do this only once, before using any other functions, it doesn't apply retroactively***

You want the size of your moving objects roughly between 0.1 and 10 meters.

For the default 32px this enables objects between 3 and 320 pixels, so this
usually needs to scale along with your game resolution and character size.

Internally, Box2D uses Meter, Kilograms and Seconds as it's main units.

#### `float AgsBox2D.GetMeter()`

Get's previously passed meter in pixels.

#### `World* AgsBox2D.CreateWorld(float gravityX, float gravityY)`

Creates a World object, this should be done before creating bodies. 

A positive gravityY is directed to ground, and a negative gravityY is directed 
upwards. 

Similarly, a positive gravityX is directed to right and a negative gravityX is 
directed to left.

#### `Body* AgsBox2D.CreateBody(World* world,  float x, float y, BodyType bodytype)`

Creates a body object in the world, at specified x and y positions. 
These positions correspond to the center of the body. 

The bodytype can be any of the types below:

- `eBodyStatic` : An object that does not move under simulation, usually the 
ground in a platformer is a static body. It doesn't collide with other static
or kinematic bodies. A static body has zero velocity. 

- `eBodyDynamic` : A fully simulated body, can collide with all body type,
 this body moves according to forces. It always has finite non-zero mass.

- `eBodyKinematic` : A kinematic body moves according it's velocity, it doesn't
move according to forces. A Kinematic body behaves as if it has infinite mass. 
It doesn't collide with other Kinematic bodies or with static bodies.

Any bodytype can be moved by user input, but you have to specific code the behavior
in AGS.

You don't need to keep the returned pointer if you aren't going to need to 
access this body anymore, since the world will hold it, but you will be unable
to destroy it unless the world is destroyed.

The specifics on a body form and mass are defined by using a Shape and Fixture.

#### `void AgsBox2D.DestroyBody(World* world,  Body* body)`

Removes a body from the world, and marks it with the property IsDestroyed true.

Always remove Joints attached to a body before removing it, otherwise the joint may turn invalid and be accessed, causing a crash.

#### `Shape* AgsBox2D.CreateRectangleShape(float w,  float h,  float x=0, float y=0)`

Creates a RectangleShape with Width w and Height h, and returns a Shape object. 

You can also change it's relative center which will be mapped to the body center.
An x of 0.0 and y of 0.0, which are defaults, maps to the shape center.

#### `Shape* AgsBox2D.CreateCircleShape(float radius,  float x=0, float y=0)`

Creates a Circle shape, and similar to RectangleShape, you can also translate
it's center.

#### `Fixture* AgsBox2D.CreateFixture(Body* body, Shape* shape, float density=0)`

Creates a Fixture, and attachs a body a shape, and specifies a density. 

You should always pass finite non-zero densities for dynamic bodies.

You don't need to keep the pointer to the shape attached to a body through a 
fixture, since the body will hold a copy of the shape. Similarly, you also
don't need to keep a pointer to the fixture, because the body will hold it too.

#### `Joint* AgsBox2D.CreateDistanceJoint(Body* bodyA, Body* bodyB, float a_x, float a_y, float b_x, float b_y, bool collideConnected = 0)`

Create Distance Joint, pass anchors on bodies A and B using world coordinates. The two bodies are assumed to be in place when this joint is created. 

This joint constrains the distance between two points on two bodies to be constant. The first anchor point is connected to the first body and the second to the second body, and the points define the length of the distance joint.

#### `Joint* AgsBox2D.CreateMotorJoint(Body* bodyA, Body* bodyB, float correction_factor,  bool collideConnected = 0)`

Create Motor Joint. This is a joint between two bodies which controls the relative motion between them.
                    
Position and rotation offsets can be specified once the MotorJoint has been created, as well as the maximum motor force and torque that will be be applied to reach the target offsets.

#### `Joint* AgsBox2D.CreateMouseJoint(Body* bodyA, float x, float y)`

Create Mouse Joint between body and a target point in the world. To make it follow the mouse, the fixed point must be updated every time-step.

The advantage of using a MouseJoint instead of just changing a body position directly is that collisions and reactions to other joints are handled by the physics engine.

#### `Joint* AgsBox2D.CreatePulleyJoint(Body* bodyA, Body* bodyB, PointF* groundAnchorA, PointF* groundAnchorB, PointF* localAnchorA, PointF* localAnchorB, float ratio, bool collideConnected = 0)`

Creates a PulleyJoint to join two bodies to each other and the ground.

The pulley joint simulates a pulley with an optional block and tackle. If the ratio parameter has a value different from one, then the simulated rope extends faster on one side than the other. In a pulley joint the total length of the simulated rope is the constant length1 + ratio * length2, which is set when the pulley joint is created.

Pulley joints can behave unpredictably if one side is fully extended. It is recommended that the method setMaxLengths  be used to constrain the maximum lengths each side can attain.

#### `void AgsBox2D.AgsBox2D.DestroyJoint(World* world,  Joint* body)`

Removes a joint from the world, it should no longer return true to isValid.

### PointF

#### `PointF* PointF.Create(float x, float y)`

Creates a PointF object with x and y values.

#### `float PointF.X`

The X coordinate property of a PointF.

#### `float PointF.Y`

The Y coordinate property of a PointF.

#### `float PointF.Length()`

Returns distance from point (X,Y) coordinates to origin (0,0).

#### `float PointF.SquaredLength()`

Returns  squared distance from point (X,Y) coordinates to origin (0,0). Slightly faster.

#### `PointF* PointF.Add(PointF* pointF)`

Returns a new point with the sum of this with pointF.

#### `PointF* PointF.Sub(PointF* pointF)`

Returns a new point with the subtraction of pointF from this.

#### `PointF* PointF.Scale(float scale)`

Returns a new point that is a copy of this point multiplied by a scalar.

#### `PointF* PointF.Rotate(float angle, float pivot_x = 0, float pivot_y = 0)`

Returns a new point with this point treated as a vector to a pivot point, rotated by an angle in radians. If you don't specify, pivot is origin (0,0).

#### `Point* PointF.ToPoint()`

Rounds this point as integer and returns a standard AGS Point object.

### Body

#### `int Body.X`

The X position property of a body as integer. 

Avoid setting this property directly. Bodies coordinates are actually float 
values in the simulation, this is provided as convenience.

#### `int Body.Y`

The Y position property of a body as integer. 

Avoid setting this property directly. Bodies coordinates are actually float 
values in the simulation, this is provided as convenience.

#### `float Body.fX`

The X position property of a body as float.

#### `float Body.fY`

The Y position property of a body as float. 

#### `float Body.Angle`

The Body Angle property. AGS can't easily rotate Sprites so avoid using angles
with bodies that you expect to map directly in someway to screen sprites.

#### `bool Body.FixedRotation`

By default, bodies are created with FixedRotation set to true. 

A body with FixedRotation set to true does not rotate, causing it's rotational 
inertia and it's inverse to be set to zero.

#### `bool Body.Bullet`

By default, bodies are created with Bullet set to false. 

Set bullet to true when the body has a small shape and moves really fast, 
this will prevent the body from having wrong collisions with thin bodies.

#### `readonly bool Body.IsDestroyed`

Returns true if it's destroyed by `AgsBox2D.DestroyBody()`.

#### `float Body.LinearDamping`

The LinearDamping property of a body. Damping occurs independently from contact
and is different than friction. Normally  the value for damping is between 
`0.0` and `0.1`.

#### `float Body.AngularDamping`

The AngularDamping property of a body, the angular drag, also happens 
independently from contact.

#### `float Body.AngularVelocity`

The AngularVelocity property of a body.

#### `float Body.Inertia`

Rotational Inertia, body's resistance to changes in angular velocity.

#### `readonly float Body.LinearVelocityX`

Gets the X vector from the body's Linear Velocity.

#### `readonly float Body.LinearVelocityY`

Gets the Y vector from the body's Linear Velocity.

#### `void Body.SetLinearVelocity(float fx, float fy)`

Set's the body LinearVelocity vector.

#### `void Body.ApplyForce(float fx, float fy)`

Applies a force on a body from it's center `0.0, 0.0` to the specified `fx, fy`
direction.

#### `void Body.ApplyAngularImpulse(float impulseIntensity)`

Applies an angular impulse on the body.

#### `void Body.ApplyLinearImpulse(float intensity_x, float intensity_y)`

Applies an impulse from the body center with the specified vector.

#### `void Body.ApplyTorque(float torque)`

Applies a torque on the body. Positive values are counter clockwise.

#### `bool Body.IsTouching(Body* otherBody)`

Returns true when a body is in contact (being touched) by other body.
This function only evaluates at the current time, so prefer using it for
resting states.

### World

The world holds all the information needed for the physics simulation. Once a
world is destroyed, the previous pointers (Bodies, Fixtures, ...) will be of no
use and you will need to recreate any objects you need in the new world.

#### `void World.Step(float dt, int velocityIteractions=8, int positionIteractions=3)`

Advances a step in the World physics simulation of `dt` seconds. 

Because AGS uses fixed game steps, a good value is  `dt = 1.0/IntToFloat(GetGameSpeed())`.

velocityIteractions and positionIteractions relates to how Box2D simulates the
world, so for information on these values I recommend looking into Box2D own documentation.

#### `int World.GetDebugSprite(int camera_x=0, int camera_y=0)`

Returns a sprite of the size of the screen with the objects in the world drawn on it.

A common usage is to create a GUI of the size of the screen and set the background
graphic of it with the sprite this function outputs.

You can pass a camera x and y value to scroll the camera on the world.

#### `FixtureArray*  World.BoundingBoxQuery(float lower_x, float lower_y, float upper_x, float upper_y)`

Returns array of fixtures which their bounding boxes are overlapped by the supplied box.

A fixture bounding box is the non rotated smallest rectangle that contains it's shape, this means a rotate rectangle or a circle, a empty area is part of the bounding box. 
This is usually good enough for a first stage of a detection, but may require additional steps.

#### `RaycastResult* World.Raycast(float x0, float y0, float x1, float y1, RaycastType rc_type = 0, FixtureArray* stopping_fixtures = 0)`

Returns RaycastResult with fixtures hit by a line, along with the hit normals. 
The raycast goes through all fixtures on the line if you supply `eRaycastPassthrough` (this is the default).

You can use `eRaycastUntilHit` for it to stop at the first fixture hit, or additionally supply an array of target fixtures so that the raycast only stops if hit any fixture on the array.

#### `readonly int World.ContactCount`

How many contacts are available. Use it to know the range to access `World.Contacts[]`. 

#### `readonly Contact* World.Contacts[]`

Gets the contacts in the world by index. These only contain fixtures in contact right now.

### Shape

#### `ShapeRectangle* Shape.AsRectangle`

If the shape is a ShapeRectangle, it returns it. Otherwise, it returns `null`.
You should not hold pointers to it, and instead access it directly like 
`myshape.AsRectangle.Width` as needed.

#### `ShapeCircle* Shape.AsCircle`

If the shape is a ShapeCircle, it returns it. Otherwise, it returns `null`.
You should not hold pointers to it, and instead access it directly like 
`myshape.AsCircle.Radius` as needed.

### Fixture

Fixtures are used when linking a shape to an object and assigning it's density.

#### `float Fixture.Density`

Density is used to compute the mass of the linked body. It's preferable to use
similar densities to all your fixtures, because this will improve the 
simulation.

#### `float Fixture.Friction`

Friction is used to make objects slide along each other realistically.

It's usually a value between `0.0` and `1.0`, but can be any non-negative value.

Box2D uses the square root of the multiplication of two contacting fixtures to 
calculate the contact friction. This means if one fixture has `0.0` friction, 
the contact will have no friction.

#### `float Fixture.Restitution`

Restitution is used to make objects bounce, and is usually a value between 
`0.0` and `1.0`. A value of `0.0` means the object won't bounce, and a value
of `1.0` means the object velocity will be exactly reflected.

#### `readonly Body* Fixture.Body`

Returns Body if it's defined for this fixture, otherwise null.

#### `int Fixture.GroupIndex`

Group the fixture belongs to, from -32768 to 32767. Fixtures with the same group will always collide if group is positive or never collide if it's negative. 
Zero means no group, and is default.

#### `int Fixture.CategoryBits`

Category of this fixture, from 16 possible categories encoded as 16-bit integer (1, 2, 4, 8, ... 32768). 65535 means all categories.

#### `int Fixture.MaskBits`

Mask of this fixture, encoded as 16-bit integer. Categories selected will collide with this fixture (ex: 5, means category 1 and 4 will collide). Default is 65535 - collide with all categories.

#### `bool Fixture.TestPoint(float x, float y)`

Returns true if a point is inside the shape of the fixture.

#### `bool Fixture.IsSensor`

Whether this fixture is a sensor. Sensors do not cause collision responses, but generate begin-contact and end-contact events.

### Joint

#### `JointDistance* Joint.AsDistance`

If this joint is a distance joint, returns the JointDistance interface; otherwise null.

#### `JointMotor* Joint.AsMotor`

If this joint is a motor joint, returns the JointMotor interface; otherwise null.

#### `JointMouse* Joint.AsMouse`

If this joint is a mouse joint, returns the JointMouse interface; otherwise null.

#### `JointPulley* Joint.AsPulley`

If this joint is a pulley joint, returns the JointPulley interface; otherwise null.

#### `bool Joint.IsValid`

If this joint is valid, returns true.

#### `bool Joint.IsActive`

If this joint is active, returns true.

#### `Body* Joint.BodyA`

Returns Body A if it's defined, otherwise null, for this joint.

#### `Body* Joint.BodyB`

Returns Body B if it's defined, otherwise null, for this joint. 

#### `JointType Joint.Type`

Returns this joint type.

### JointDistance

#### `float JointDistance.Length`

The equilibrium distance between the two Bodies. 

#### `float JointDistance.DampingRatio`

The damping ratio, typically between 0 and 1. At 1, the damping is critical.

#### `float JointDistance.Frequency`

The frequency of a harmonic oscillator. Should be smaller than half the frame rate. 

### JointMotor

#### `void JointMotor.SetLinearOffset(float fx, float fy)`

Sets the target linear offset between the two bodies the joint is attached to.

#### `float JointMotor.LinearOffsetX`

The target linear offset X axis between the two bodies the joint is attached to.

#### `float JointMotor.LinearOffsetY`

The target linear offset Y axis between the two bodies the joint is attached to.

#### `float JointMotor.AngularOffset`

The target angular offset between the two bodies the joint is attached to.

#### `float JointMotor.MaxForce`

The Maximum Force applied to reach target position.

#### `float JointMotor.MaxTorque`

The Maximum Torque applied to reach target rotation.

### JointMouse

#### `void JointMouse.SetTarget(float fx, float fy)`

Sets the target point.

#### `float JointMouse.TargetX`

The target point X axis.

#### `float JointMouse.TargetY`

The target point Y axis.

#### `float JointMouse.DampingRatio`

The damping ratio, typically between 0 and 1. At 1, the damping is critical.

#### `float JointMouse.Frequency`

The frequency of a harmonic oscillator. Should be smaller than half the frame rate.

#### `float JointMouse.MaxForce`

The Maximum Force applied to reach target position.

### JointPulley

#### `float JointPulley.LengthA`

The current length of the segment attached to the first body. 

#### `float JointPulley.LengthB`

The current length of the segment attached to the second body.

#### `float JointPulley.Ratio`

The pulley ratio.

### Contact

#### `readonly bool Contact.IsValid`

Whether the Contact is still valid. 

#### `readonly PointF* Contact.Normal`

The normal vector between two shapes that are in contact.

#### `readonly PointF* Contact.Positions[]`

The contact points of the two colliding fixture, use PositionsCount to find out how many. Index starts at 0. 

#### `readonly int Contact.PositionsCount`

How many position of contact are available.

#### `readonly Fixture* Contact.FixtureA`

One of the Fixtures that hold the shapes in contact.

#### `readonly Fixture* Contact.FixtureB`

The other of the Fixtures that hold the shapes in contact.

#### `bool Contact.Enabled`

Whether the contact is enabled.

#### `float Contact.Restitution`

The restitution between two shapes that are in contact.

#### `float Contact.Friction`

The friction between two shapes that are in contact.

---

## Download AgsBox2D

This plugin is available as `agsbox2d.dll` under [assets, in the latest release](https://github.com/ericoporto/agsbox2d/releases/latest/),
for usage with Windows and the AGS Editor. You can also find it there built 
for Linux as `libagsbox2d.so` and for MacOS as `libagsbox2d.dylib`.

## Building agsbox2d 

AgsBox2D both Makefile and the VS Solution file, expects to find [Adventure Game Studio source code](https://github.com/adventuregamestudio/ags)
in a folder `../ags/`. After you do this, you need to clone this source code.

```
  ~/git/ags/
  ~/git/agsbox2d/
```

Navigate then to the directory where you cloned this repository.

On Windows, you can load the solution on the root of it's directory and load 
it on Visual Studio. It should work with VS 2015, 2017 and 2019. You will need
v140 tools (VS should promptly warn you to install if you don't have it).
The dll provided by Release Win32 builds is the one you should build to use
with an AGS Game at the time of this writing.

On Linux and MacOS, navigate to `agsbox2d/` inside the directory and type `make`.

## License and Author

AgsBox2D is made by Érico Vieira Porto provided with Z-Lib [LICENSE](LICENSE).

Box2D itself is made by Erin Catto and is provided with a Z-Lib [LICENSE](Box2D/Box2D/LICENSE) too.

Magic Cliffs background used on the demo is made by [ansimuz](https://opengameart.org/content/magic-cliffs-environment) and is available as CC-BY 3.0.
