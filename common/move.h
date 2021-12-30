#ifndef AGV_MOVE_H
#define AGV_MOVE_H

#include <vector>

#define MOVE_INSTRUCTION_TYPE_SPEED 1
#define MOVE_INSTRUCTION_TYPE_TARGET 2
#define MOVE_INSTRUCTION_TYPE_TURN 3
#define MOVE_INSTRUCTION_TYPE_COLLISION 4
#define MOVE_INSTRUCTION_TYPE_LIFT 5
#define MOVE_INSTRUCTION_TYPE_LIFT_HEIGHT 6
#define MOVE_INSTRUCTION_TYPE_MOTION_MODE 7

struct movement_instruction_target
{
    float start_x;
    float start_y;
    float start_theta;
    float x;
    float y;
    float theta;
	int dir;
};
struct movement_instruction_turn
{
    int dir;
    float angle;
    float angle_speed;
};
struct movement_instruction_speed
{
    int speed;
    int turn_left;
};
struct movement_instruction_collision
{
    int level;
};
struct movement_instruction_lift
{
    int dir;
	int height;
};
/**************************************************四舵轮***************************************************/
struct movement_instruction_motion_mode{
    int run_state;
    int angle;
};
/************************************************************************************************************/
class MoveInstruction
{
public:
    int type;
    struct movement_instruction_speed speed;
    struct movement_instruction_target target;
    struct movement_instruction_turn turn;
	struct movement_instruction_collision collision;
    struct movement_instruction_lift lift;
/****************************************************************四舵轮*******************************************************/
    struct movement_instruction_motion_mode motion_mode;
/************************************************************************************************************/
/****************************************************************加上motion_mode*******************************************************/
    void to_char_array(std::vector<char> *output)
{
    output->reserve(output->size() + sizeof(int) + sizeof(speed) + sizeof(target) + sizeof(turn) + sizeof(collision) + sizeof(lift)+sizeof(motion_mode));
    char *p = (char *)(&type);
    for (int i = 0; i < sizeof(int); i++)
    {
        output->push_back(p[i]);
    }
    p = (char *)(&speed);
    for (int i = 0; i < sizeof(speed); i++)
    {
        output->push_back(p[i]);
    }
    p = (char *)(&target);
    for (int i = 0; i < sizeof(target); i++)
    {
        output->push_back(p[i]);
    }
    p = (char *)(&turn);
    for (int i = 0; i < sizeof(turn); i++)
    {
        output->push_back(p[i]);
    }
    p = (char *)(&collision);
    for (int i = 0; i < sizeof(collision); i++)
    {
        output->push_back(p[i]);
    }
    p = (char *)(&lift);
    for (int i = 0; i < sizeof(lift); i++)
    {
        output->push_back(p[i]);
    }
    p = (char *)(&motion_mode);
    for (int i = 0;i<sizeof(motion_mode);i++)
    {
        output->push_back(p[i]);
    }
}
    void from_char_array(char *buf, int size)
{
    if (size >= sizeof(int) + sizeof(speed) + sizeof(target) + sizeof(turn) + sizeof(collision) + sizeof(lift)+sizeof(motion_mode))
    {
        type = *((int *)buf);
        speed = *((struct movement_instruction_speed *)(buf + sizeof(int)));
        target = *((struct movement_instruction_target *)(buf + sizeof(int) + sizeof(speed)));
        turn = *((struct movement_instruction_turn *)(buf + sizeof(int) + sizeof(speed) + sizeof(target)));
        collision = *((struct movement_instruction_collision *)(buf + sizeof(int) + sizeof(speed) + sizeof(target) + sizeof(turn)));
        lift = *((struct movement_instruction_lift *)(buf + sizeof(int) + sizeof(speed) + sizeof(target) + sizeof(turn) + sizeof(collision)));
        motion_mode=*((struct movement_instruction_motion_mode *)(buf + sizeof(int) + sizeof(speed) + sizeof(target) + sizeof(turn) + sizeof(collision)+sizeof(motion_mode)));
    }
}
};

#endif
