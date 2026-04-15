#include "communication/vci_can_adapter.h"
#include "communication/controlcan.h"
#include <dlfcn.h>
#include <cstring>

namespace dac {

namespace {

bool timingFromKbps(int kbps, UCHAR &timing0, UCHAR &timing1)
{
    switch (kbps) {
    case 20:   timing0 = 0x31; timing1 = 0x1C; return true;
    case 50:   timing0 = 0x09; timing1 = 0x1C; return true;
    case 100:  timing0 = 0x04; timing1 = 0x1C; return true;
    case 125:  timing0 = 0x03; timing1 = 0x1C; return true;
    case 250:  timing0 = 0x01; timing1 = 0x1C; return true;
    case 500:  timing0 = 0x00; timing1 = 0x1C; return true;
    case 1000: timing0 = 0x00; timing1 = 0x14; return true;
    default:   return false;
    }
}

} // namespace

VciCanAdapter::VciCanAdapter() = default;
VciCanAdapter::~VciCanAdapter() { close(); unloadLibrary(); }
bool VciCanAdapter::loadLibrary() {
    if (libHandle_) return true;
    libHandle_ = dlopen("/home/yang/DualArmRobot/dual_arm_control/lib/libcontrolcan.so", RTLD_NOW);
    if (!libHandle_) return false;
    fn_OpenDevice_ = (FN_OpenDevice)dlsym(libHandle_, "VCI_OpenDevice");
    fn_CloseDevice_ = (FN_CloseDevice)dlsym(libHandle_, "VCI_CloseDevice");
    fn_InitCAN_ = (FN_InitCAN)dlsym(libHandle_, "VCI_InitCAN");
    fn_StartCAN_ = (FN_StartCAN)dlsym(libHandle_, "VCI_StartCAN");
    fn_ResetCAN_ = (FN_ResetCAN)dlsym(libHandle_, "VCI_ResetCAN");
    fn_Transmit_ = (FN_Transmit)dlsym(libHandle_, "VCI_Transmit");
    fn_Receive_ = (FN_Receive)dlsym(libHandle_, "VCI_Receive");
    fn_GetReceiveNum_ = (FN_GetReceiveNum)dlsym(libHandle_, "VCI_GetReceiveNum");
    return fn_OpenDevice_ && fn_CloseDevice_ && fn_InitCAN_ && fn_StartCAN_ &&
           fn_ResetCAN_ && fn_Transmit_ && fn_Receive_ && fn_GetReceiveNum_;
}
void VciCanAdapter::unloadLibrary() { if (libHandle_) { dlclose(libHandle_); libHandle_ = nullptr; } }
bool VciCanAdapter::open(const BusConfig &cfg) {
    if (!loadLibrary()) return false;
    deviceInd_ = cfg.devicePath.toUInt();
    canChannel_ = cfg.canChannel;

    UCHAR timing0 = 0;
    UCHAR timing1 = 0;
    if (!timingFromKbps(cfg.canBitrate, timing0, timing1)) {
        return false;
    }

    if (fn_OpenDevice_(deviceType_, deviceInd_, 0) != 1) return false;
    VCI_INIT_CONFIG config{};
    config.AccCode = 0x00000000;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 1; // 双滤波
    config.Timing0 = timing0;
    config.Timing1 = timing1;
    config.Mode = 0; // 正常模式
    if (fn_InitCAN_(deviceType_, deviceInd_, canChannel_, &config) != 1) {
        fn_CloseDevice_(deviceType_, deviceInd_);
        return false;
    }
    if (fn_StartCAN_(deviceType_, deviceInd_, canChannel_) != 1) {
        fn_CloseDevice_(deviceType_, deviceInd_);
        return false;
    }
    opened_ = true; return true;
}
void VciCanAdapter::close() {
    if (opened_) { fn_ResetCAN_(deviceType_, deviceInd_, canChannel_); fn_CloseDevice_(deviceType_, deviceInd_); opened_ = false; }
}
bool VciCanAdapter::isOpen() const { return opened_; }
bool VciCanAdapter::sendFrame(const CanFrame &frame) {
    if (!opened_) return false;
    VCI_CAN_OBJ obj{};
    obj.ID = frame.canId;
    obj.SendType = 0;
    obj.RemoteFlag = 0;
    obj.ExternFlag = 0;
    obj.DataLen = static_cast<BYTE>(frame.dlc);
    std::memcpy(obj.Data, frame.data, 8);
    return fn_Transmit_(deviceType_, deviceInd_, canChannel_, &obj, 1) == 1;
}
void VciCanAdapter::processIncoming() {
    if (!opened_) return;
    uint32_t count = fn_GetReceiveNum_(deviceType_, deviceInd_, canChannel_);
    if (count == 0) return;
    VCI_CAN_OBJ objs[256]{};
    uint32_t received = fn_Receive_(deviceType_, deviceInd_, canChannel_, objs, 256, 0);
    QMutexLocker lk(&mutex_);
    for (uint32_t i = 0; i < received; ++i) {
        const auto &obj = objs[i];
        CanFrame f;
        f.canId = obj.ID;
        f.dlc = obj.DataLen;
        std::memcpy(f.data, obj.Data, 8);
        f.sequence = ++sequence_;
        frameQueue_.push_back(f);
    }
    if (received > 0) frameCond_.wakeAll();
}
bool VciCanAdapter::waitForFrame(CanFrame &out, int timeoutMs) {
    QMutexLocker lk(&mutex_);
    if (frameQueue_.empty() && !frameCond_.wait(&mutex_, timeoutMs)) return false;
    if (frameQueue_.empty()) return false;
    out = frameQueue_.front(); frameQueue_.pop_front(); return true;
}
uint64_t VciCanAdapter::currentSequence() const { QMutexLocker lk(&mutex_); return sequence_; }
} // namespace dac
