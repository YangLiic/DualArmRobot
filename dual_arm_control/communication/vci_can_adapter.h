#pragma once
#include "communication/ican_adapter.h"
#include <deque>

namespace dac {

class VciCanAdapter : public ICanAdapter
{
public:
    VciCanAdapter();
    ~VciCanAdapter() override;
    bool open(const BusConfig &cfg) override;
    void close() override;
    bool isOpen() const override;
    bool sendFrame(const CanFrame &frame) override;
    void processIncoming() override;
    bool waitForFrame(CanFrame &out, int timeoutMs) override;
    uint64_t currentSequence() const override;
    QString adapterName() const override { return QStringLiteral("VCI (CANalyst-II)"); }

private:
    bool loadLibrary();
    void unloadLibrary();
    bool opened_ = false;
    uint32_t deviceType_ = 4;
    uint32_t deviceInd_ = 0;
    uint32_t canChannel_ = 0;
    mutable QMutex mutex_;
    QWaitCondition frameCond_;
    std::deque<CanFrame> frameQueue_;
    uint64_t sequence_ = 0;
    void *libHandle_ = nullptr;

    typedef uint32_t (*FN_OpenDevice)(uint32_t, uint32_t, uint32_t);
    typedef uint32_t (*FN_CloseDevice)(uint32_t, uint32_t);
    typedef uint32_t (*FN_InitCAN)(uint32_t, uint32_t, uint32_t, void*);
    typedef uint32_t (*FN_StartCAN)(uint32_t, uint32_t, uint32_t);
    typedef uint32_t (*FN_ResetCAN)(uint32_t, uint32_t, uint32_t);
    typedef uint32_t (*FN_Transmit)(uint32_t, uint32_t, uint32_t, void*, uint32_t);
    typedef uint32_t (*FN_Receive)(uint32_t, uint32_t, uint32_t, void*, uint32_t, int);
    typedef uint32_t (*FN_GetReceiveNum)(uint32_t, uint32_t, uint32_t);

    FN_OpenDevice fn_OpenDevice_ = nullptr;
    FN_CloseDevice fn_CloseDevice_ = nullptr;
    FN_InitCAN fn_InitCAN_ = nullptr;
    FN_StartCAN fn_StartCAN_ = nullptr;
    FN_ResetCAN fn_ResetCAN_ = nullptr;
    FN_Transmit fn_Transmit_ = nullptr;
    FN_Receive fn_Receive_ = nullptr;
    FN_GetReceiveNum fn_GetReceiveNum_ = nullptr;
};

} // namespace dac
