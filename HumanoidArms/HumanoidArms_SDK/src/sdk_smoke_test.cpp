#include "head.h"

using namespace std;

namespace {

ArmSide parseArm(const string &value) {
    if (value == "left" || value == "LEFT" || value == "l") {
        return LEFT_ARM;
    }
    if (value == "right" || value == "RIGHT" || value == "r") {
        return RIGHT_ARM;
    }

    cerr << "Unknown arm: " << value << ". Use left or right." << endl;
    exit(2);
}

void printUsage(const char *argv0) {
    cout << "Usage: " << argv0 << " [left|right] [device_index] [can_index]\n";
    cout << "Example: " << argv0 << " left 0 1\n";
}

} // namespace

int main(int argc, char **argv) {
    if (argc > 1 && string(argv[1]) == "--help") {
        printUsage(argv[0]);
        return 0;
    }

    ArmSide side = LEFT_ARM;
    int device_index = 0;
    int can_index = 1;

    if (argc > 1) {
        side = parseArm(argv[1]);
    }
    if (argc > 2) {
        device_index = stoi(argv[2]);
    }
    if (argc > 3) {
        can_index = stoi(argv[3]);
    }

    cout << "SDK smoke test starting..." << endl;
    cout << "Local IP: " << ip_address() << endl;
    view_robot_config_information();

    can_init();

    MatrixXd joint = get_joint(side, device_index, can_index);
    MatrixXd pos = get_Pos(side, device_index, can_index);

    cout << "\nJoint angles (rad):\n" << joint << endl;
    cout << "\nJoint angles (deg):\n" << joint * deg << endl;
    cout << "\nTCP pose [x y z rx ry rz]:\n" << pos << endl;

    Exit();
    cout << "\nSDK smoke test completed." << endl;
    return 0;
}
