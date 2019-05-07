#ifndef ROVER_H
#define ROVER_H

void wait_for_command();
void parse_command();
void turn();
void move_forward();
void move_reverse();
bool check_ir();

#endif //ROVER_H