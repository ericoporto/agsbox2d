# agsbox2d

[![Build Status](https://dev.azure.com/ericoporto/agsbox2d/_apis/build/status/ericoporto.agsbox2d?branchName=master)](https://dev.azure.com/ericoporto/agsbox2d/_build/latest?definitionId=11&branchName=master)

AgsBox2D is a physics plugin for Adventure Game Studio that gives access to the Box2D library created by Erin Catto.
Because I never used Box2D directly before, I tried to make the API similar to Love physics module.

AgsBox2D is still in early development.

- [In development warning](#in-development-warning)
- [Usage example](#usage-example)
- [Script API](#script-api)
  * [AgsBox2D](#agsbox2d)
  * [Body](#body)
  * [World](#world)
  * [Shape](#shape)
  * [Fixture](#fixture)
- [License and Author](#license-and-author)

## In development warning

AgsBox2D is in development and doesn't support many AGS features (you can't 
save and load with regular AGS save methods yet using this plugin for now).

Still, if you want to experiment with it and report your findings, this README
should prove itself useful and I appreciate any help in making this plugin work
well with AGS.

## Usage example

Below we will do a very simple example that creates a ground, and adds two
boxes and a ball. The ball is controlled by keyboard input.

This example uses a room script and a gui named gDebugGui, with size 320x180 
and border and color set to 0, used to print the example objects. These are not
AGS Objects.

```AGS Script
// example room script file
DynamicSprite* dynspr;
DrawingSurface* surf;

World* world;

struct Physical {
Body* body;
Shape* shape;
Fixture* fixture;
};

Physical ground;
Physical ball;
Physical box1;
Physical box2;

void initPhysics(){
  AgsBox2D.SetMeter(32.0);
  world = AgsBox2D.CreateWorld(0.0, 9.8*AgsBox2D.GetMeter());
  
  ground.body = AgsBox2D.CreateBody(world, 160.0, 160.0, eBodyStatic);
  ground.shape = AgsBox2D.CreateRectangleShape(320.0, 40.0);
  ground.fixture = AgsBox2D.CreateFixture(ground.body, ground.shape);

  ball.body = AgsBox2D.CreateBody(world, 160.0, 40.0, eBodyDynamic);
  ball.shape = AgsBox2D.CreateCircleShape(20.0);
  ball.fixture = AgsBox2D.CreateFixture(ball.body, ball.shape, 1.0);  
  
  box1.body = AgsBox2D.CreateBody(world, 80.0, 60.0, eBodyDynamic);
  box1.shape = AgsBox2D.CreateRectangleShape(30.0, 20.0);
  box1.fixture = AgsBox2D.CreateFixture(box1.body, box1.shape, 5.0);
  
  box2.body = AgsBox2D.CreateBody(world, 80.0, 80.0, eBodyDynamic);
  box2.shape = AgsBox2D.CreateRectangleShape(20.0, 20.0);
  box2.fixture = AgsBox2D.CreateFixture(box2.body, box2.shape, 2.0);
}

int DebugDraw(){
  if(dynspr!=null){
    dynspr.Delete();
    dynspr = null;
  }
  
  dynspr = DynamicSprite.Create(320, 180, true);
  surf = dynspr.GetDrawingSurface();
  
  surf.DrawingColor = 0; //BLACK
  surf.DrawRectangle(0, 0, 320, 180);
  
  surf.DrawingColor = 4064; //GREEN
  surf.DrawRectangle(ground.body.X-ground.shape.AsRectangle.Width/2, 
                     ground.body.Y-ground.shape.AsRectangle.Height/2,
                     ground.body.X+ground.shape.AsRectangle.Width/2, 
                     ground.body.Y+ground.shape.AsRectangle.Height/2);
  
  surf.DrawingColor = 63808; //RED
  surf.DrawCircle(ball.body.X, ball.body.Y, 20);
  
  surf.DrawingColor = 63808; //RED
  surf.DrawRectangle(box1.body.X-15, box1.body.Y-10, box1.body.X+15, box1.body.Y+10);  
  
  surf.DrawingColor = 63808; //RED
  surf.DrawRectangle(box2.body.X-10, box2.body.Y-10, box2.body.X+10, box2.body.Y+10);
  
  surf.Release();
  return dynspr.Graphic;
}

function room_Load()
{
  initPhysics();
}

function room_RepExec() {
  gDebugGui.BackgroundGraphic = DebugDraw();
  
  if(IsKeyPressed(eKeyLeftArrow)){
    ball.body.ApplyForce(-500.0, 0.0);
  }
  if(IsKeyPressed(eKeyRightArrow)){
    ball.body.ApplyForce(500.0, 0.0);
  }
  if(IsKeyPressed(eKeyUpArrow)){
    ball.body.ApplyForce(0.0, -5000.0);
    ball.body.SetLinearVelocity(0.0, 0.0);
  }
  if(IsKeyPressed(eKeyDownArrow)){
    ball.body.ApplyForce(0.0, 500.0);
  }
  
  world.Step(1.0/IntToFloat(GetGameSpeed()), 8, 3);
}
```

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

#### `float Body.LinearDamping`

The LinearDamping property of a body. Damping occurs independently from contact
and is different than friction. Normally  the value for damping is between 
`0.0` and `0.1`.

#### `readonly float Body.LinearVelocityX`

Gets the X vector from the body's Linear Velocity.

#### `readonly float Body.LinearVelocityY`

Gets the Y vector from the body's Linear Velocity.

#### `void Body.ApplyForce(float fx, float fy)`

Applies a force on a body from it's center `0.0, 0.0` to the specified `fx, fy`
direction.

#### `void Body.SetLinearVelocity(float fx, float fy)`

Set's the body LinearVelocity vector.

#### `void Body.ApplyAngularImpulse(float impulseIntensity)`

Applies an angular impulse on the body.

### World

The world holds all the information needed for the physics simulation. Once a
world is destroyed, the previous pointers (Bodies, Fixtures, ...) will be of no
use and you will need to recreate any objects you need in the new world.

#### `void World.Step(float dt, int velocityIteractions = 8, int positionIteractions = 3)`

Advances a step in the World physics simulation of `dt` seconds. 

Because AGS uses fixed game steps, a good value is  `dt = 1.0/IntToFloat(GetGameSpeed())`.

velocityIteractions and positionIteractions relates to how Box2D simulates the
world, so for information on these values I recommend looking into Box2D own documentation.

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

For now, fixtures have no accessible properties or methods and are only used
when linking a shape to an object and assigning it's density.

## License and Author

AgsBox2D is made by Érico Vieira Porto provided with Z-Lib [LICENSE](LICENSE).

Box2D itself is made by Erin Catto and is provided with a Z-Lib [LICENSE](Box2D/Box2D/LICENSE) too.
