#pragma once
#include "models/common_types.h"
#include <QString>
#include <deque>
#include <QMutex>
#include <QWaitCondition>

namespace dac {

class ICanAdapter
{
public:
    virtual ~ICanAdapter() = default;
    virtual bool open(const BusConfig &cfg) = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;
    virtual bool sendFrame(const CanFrame &frame) = 0;
    virtual void processIncoming() = 0;
    virtual bool waitForFrame(CanFrame &out, int timeoutMs) = 0;
    virtual uint64_t currentSequence() const = 0;
    virtual QString adapterName() const = 0;
};

} // namespace dac
