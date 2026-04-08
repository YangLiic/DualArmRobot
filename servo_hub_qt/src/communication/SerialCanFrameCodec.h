#pragma once

#include "models/Types.h"

#include <QByteArray>
#include <QList>

class SerialCanFrameCodec
{
public:
    static QByteArray encode(quint32 canId, const QByteArray &payload);
    static QList<CanFrame> parse(QByteArray &buffer, quint64 &sequenceCounter);
};

