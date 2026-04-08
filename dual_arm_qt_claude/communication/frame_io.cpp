/**
 * FrameIO - USB-CAN 适配器底层帧收发实现
 */
#include "frame_io.h"

#include <QDebug>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>

namespace dar {

FrameIO::FrameIO(QObject *parent)
    : QObject(parent)
{
}

FrameIO::~FrameIO()
{
    close();
}

bool FrameIO::open(const QString &portName, int baudRate)
{
    if (serialFd_ >= 0) {
        close();
    }

    portName_ = portName;
    baudRate_ = baudRate;

    serialFd_ = openSerial(portName, baudRate);
    if (serialFd_ < 0) {
        emit errorOccurred(QStringLiteral("无法打开串口: %1").arg(portName));
        return false;
    }

    emit portOpened();
    return true;
}

void FrameIO::close()
{
    stopReceive();
    if (serialFd_ >= 0) {
        ::close(serialFd_);
        serialFd_ = -1;
        emit portClosed();
    }
}

bool FrameIO::isOpen() const
{
    return serialFd_ >= 0;
}

bool FrameIO::sendFrame(uint32_t canId, const uint8_t *data, uint8_t dlc)
{
    if (serialFd_ < 0) {
        return false;
    }

    uint8_t frame[17];
    frame[0] = 0xAA;
    frame[1] = 0x00;
    frame[2] = 0x00;
    frame[3] = dlc;
    frame[4] = 0x00;
    frame[5] = 0x00;
    frame[6] = (canId >> 8) & 0xFF;
    frame[7] = canId & 0xFF;

    std::memcpy(&frame[8], data, dlc);
    if (dlc < 8) {
        std::memset(&frame[8 + dlc], 0, 8 - dlc);
    }
    frame[16] = 0x7A;

    ssize_t written = ::write(serialFd_, frame, 17);
    return written == 17;
}

void FrameIO::startReceiveLoop()
{
    if (serialFd_ < 0) return;
    receiving_ = true;

    uint8_t buffer[1024];
    fd_set rdfs;
    struct timeval timeout;

    while (receiving_) {
        FD_ZERO(&rdfs);
        FD_SET(serialFd_, &rdfs);

        timeout.tv_sec = 0;
        timeout.tv_usec = 50000;  // 50ms

        int ret = select(serialFd_ + 1, &rdfs, nullptr, nullptr, &timeout);
        if (ret < 0) {
            if (!receiving_) break;
            emit errorOccurred(QStringLiteral("select() 错误"));
            break;
        } else if (ret == 0) {
            continue;
        }

        if (FD_ISSET(serialFd_, &rdfs)) {
            ssize_t nbytes = ::read(serialFd_, buffer, sizeof(buffer));
            if (nbytes > 0) {
                size_t processed = 0;
                while (processed < static_cast<size_t>(nbytes)) {
                    int parsed = parseFrame(&buffer[processed],
                                           static_cast<size_t>(nbytes) - processed);
                    if (parsed > 0) {
                        processed += parsed;
                    } else {
                        break;
                    }
                }
            }
        }
    }
}

void FrameIO::stopReceive()
{
    receiving_ = false;
}

bool FrameIO::waitForFrame(uint64_t lastSeq, CanFrame &frame, int timeoutMs)
{
    QMutexLocker lock(&frameMutex_);

    // 先检查队列中是否已有新帧
    auto checkQueue = [&]() -> bool {
        while (!frameQueue_.isEmpty()) {
            CanFrame f = frameQueue_.dequeue();
            if (f.sequence > lastSeq) {
                frame = f;
                return true;
            }
        }
        return false;
    };

    if (checkQueue()) return true;

    // 等待通知
    if (frameCv_.wait(&frameMutex_, timeoutMs)) {
        return checkQueue();
    }

    return false;
}

uint64_t FrameIO::currentSequence() const
{
    QMutexLocker lock(&frameMutex_);
    return frameSequence_;
}

int FrameIO::openSerial(const QString &portName, int baudRate)
{
    int fd = ::open(portName.toLocal8Bit().constData(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        ::close(fd);
        return -1;
    }

    speed_t speed;
    switch (baudRate) {
        case 9600:    speed = B9600;    break;
        case 115200:  speed = B115200;  break;
        case 230400:  speed = B230400;  break;
        case 460800:  speed = B460800;  break;
        case 921600:  speed = B921600;  break;
        case 2000000: speed = B2000000; break;
        default:      speed = B9600;    break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~(OPOST | ONLCR);

    tty.c_cc[VTIME] = 1;  // 100ms
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

int FrameIO::parseFrame(const uint8_t *buffer, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        if (buffer[i] == 0xAA && (i + 16) < len && buffer[i + 16] == 0x7A) {
            CanFrame frame;
            frame.dlc = buffer[i + 3];
            frame.can_id = (static_cast<uint32_t>(buffer[i + 6]) << 8) | buffer[i + 7];
            std::memcpy(frame.data, &buffer[i + 8], 8);

            {
                QMutexLocker lock(&frameMutex_);
                frameSequence_++;
                frame.sequence = frameSequence_;
                frameQueue_.enqueue(frame);
                if (frameQueue_.size() > 256) {
                    frameQueue_.dequeue();
                }
            }
            frameCv_.wakeAll();

            emit frameReceived(frame);

            return static_cast<int>(i + 17);
        }
    }
    return 0;
}

}  // namespace dar
