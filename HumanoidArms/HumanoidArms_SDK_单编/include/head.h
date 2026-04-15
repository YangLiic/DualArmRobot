#ifndef HEAD_H
#define HEAD_H

#include <Eigen/Dense>
#include <vector>
#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <csignal>
#include <mutex>
#include <shared_mutex>
#include <map>
#include <termios.h>
#include <fcntl.h>
#include <algorithm>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <thread>
#include <array>
#include <iomanip>
#include <dirent.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/types.h>
// #include <ceres/ceres.h>
#include <string>
#include <unordered_map>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdint>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <atomic>

#include "function.h"
#include "controlcan.h"
#include "Can_Box.h"
#include "Kinematics.h"
#include "Ti5BASIC.h"

inline int stop;


#endif