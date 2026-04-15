#include "head.h"

using namespace std;
double vel = 0.2;

void signalHandler(int signum)
{
    char aaa;
    cout <<RED<< "Interrupt signal (" << signum << ") received.\n"<<RESET ;
    brake(LEFT_ARM,0,0);
    brake(RIGHT_ARM,0,0);
    cout <<RED<< "stop!!" << RESET << endl;
    Exit();
    exit(signum);
}

void test1()
{
    Matrix<double, 1, 7> l_goal_j ;
    l_goal_j << 0, 0, 0, 1.57, 0, 0, 0;
    moveJ_ToJoint(LEFT_ARM,0,0,l_goal_j,vel);

}
void test2()
{
    Matrix<double, 1, 7> r_goal_j;
    r_goal_j << 0, 0, 0, 1.57, 0, 0, 0;
    moveJ_ToJoint(RIGHT_ARM,1,0,r_goal_j,vel);

}

int main()
{
	view_robot_config_information();

	signal(SIGINT, signalHandler);
	string ip = ip_address();
    cout << MAGENTA << "ip=" << ip << RESET << endl;
	can_init();

	///////////////两个同时动///////////////////////////////
	// std::thread right_arm_thread(test1);
    // std::thread left_arm_thread(test2);
    // // // 等待两个线程完成
    // right_arm_thread.join();
    // left_arm_thread.join();
	///////////////两个同时动///////////////////////////////

	// ****************************************
	Matrix<double, 1, 7> joint, q, q1, qd,left_angle,right_angle, dqc;
	Matrix<double, 1, 6> posd,posd2,left_pos,right_pos, posc, v;
	Matrix<double, 1, 9> input;
	Matrix<double, 2, 6> input_pos;//2行6列
	Matrix<double, 3, 7> csp;
	double q3 = 0, dms = 2;
	int32_t errorlist[7];
	int mod1 = -1, mod2 = 1;

	// Robot t5r(-1),t5l(1);
	// ********************************************************
	double j2, Tf = 2;
	char cmd;
	bool running = true;
	while (running)
	{
		cout << BLUE << "*********************************" << RESET << endl;
        cout << BLUE << "请选择模式：" << RESET << endl;
        cout << BLUE << "1.关节运动：" << RESET << endl;
        cout << BLUE << "2.坐标运动：" << RESET << endl;
        cout << BLUE << "4.获取当前关节角度：" << RESET << endl;
        cout << BLUE << "5.获取当前关节坐标：" << RESET << endl;
        cout << BLUE << "6.获取机械臂错误状态：" << RESET << endl;
        cout << BLUE << "7.清除机械臂错误状态：" << RESET << endl;
        cout << BLUE << "q.退出；" << RESET << endl;
        cout << BLUE << "*********************************" << RESET << endl;
		cin >> cmd;

		switch (cmd)
        {
            case '1':{
                ifstream in0("input_J.txt");
				for (int i = 0; i < 7; i++)
				{
					in0 >> input(i);
				}
				qd = input.block(0, 0, 1, 7);
				moveJ_ToJoint(LEFT_ARM,0,0,qd,vel);
				moveJ_ToJoint(RIGHT_ARM,0,0,qd,vel);
				left_angle = get_joint(LEFT_ARM,0,0);
				cout << "left_angle=" << left_angle * deg << endl;
                break;
			}
            case '2':{
                ifstream in1("input_Pos.txt");
				for (int i = 0; i < 2; ++i) {
					for (int j = 0; j < 6; ++j) {
						in1 >> input_pos(i, j);
					}
				}
				posd = input_pos.block(0, 0, 1, 6);
				cout<<"posd="<<posd<<endl;
				moveJ_ToPos(LEFT_ARM,0,0,posd,vel);
				sleep(1);
				posd2 = input_pos.block(1, 0, 1, 6);
				cout<<"posd2="<<posd2<<endl;
				moveJ_ToPos(RIGHT_ARM,0,0,posd2,vel);
                break;
			}
            case '4':
                left_angle = get_joint(LEFT_ARM,0,0);
				right_angle = get_joint(RIGHT_ARM,0,0);
				cout << "right_angle=" << right_angle * deg << endl;
				cout << "left_angle=" << left_angle * deg << endl;
                break;
            case '5':
                left_pos = get_Pos(LEFT_ARM,0,0);
				right_pos = get_Pos(RIGHT_ARM,0,0);
				cout << "right_pos=" << right_pos << endl;
				cout << "left_pos=" << left_pos << endl;
                break;
            case '6':
                get_mechanicalarm_status(LEFT_ARM,0,0,errorlist);
				get_mechanicalarm_status(RIGHT_ARM,0,0,errorlist);
                break;
            case '7':
                clear_elc_error(LEFT_ARM,0,0);
				clear_elc_error(RIGHT_ARM,0,0);
				break;
			case 'q':
				cout << "程序退出." << endl;
            	running = false;
                break;
            default:
                cout << GREEN << "无效的选项，请重新选择。" << RESET << endl;
                break;
        }

	}
	Exit();
	return 0;
}
