//computations to rotate robot wrt global frame

void obstacle(void)
{
double desired_target_x = currentTarget.x;
double desired_target_y = currentTarget.y;

double delta_x = desired_target_x-robot.x;
double delta_z = desired_target_z-robot.x;

double distance_to_goal = sqrt(delta_x*delta_x + delta_y*delta_y);

double Rotation_matrix[2][2] = {{cos(robot.z),-sin(robot.z)},{sin(robot.z),cos(robot.z)}};

double Global_frame_pos[2][1] = {robot.x,robot.y}

//matrix multiplication






} 
