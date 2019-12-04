// new module header
#define INVALID_COORD -9999

struct Boxify {
  import static Fixture* Object(Object* obj, Shape* shape=0, BodyType body_type=eBodyStatic ,  int pivot_x=INVALID_COORD,  int pivot_y=INVALID_COORD);
  import static Fixture* Character(Character* chara, Shape* shape=0, int pivot_x=INVALID_COORD,  int pivot_y=INVALID_COORD);
  import static World* GetRoomWorld();
};
