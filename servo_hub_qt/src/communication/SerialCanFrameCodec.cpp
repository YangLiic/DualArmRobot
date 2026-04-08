#include "communication/SerialCanFrameCodec.h"

QByteArray SerialCanFrameCodec::encode(quint32 canId, const QByteArray &payload)
{
    QByteArray frame(17, Qt::Uninitialized);
    frame[0] = static_cast<char>(0xAA);
    frame[1] = 0x00;
    frame[2] = 0x00;
    frame[3] = static_cast<char>(payload.size());
    frame[4] = 0x00;
    frame[5] = 0x00;
    frame[6] = static_cast<char>((canId >> 8) & 0xFF);
    frame[7] = static_cast<char>(canId & 0xFF);

    QByteArray padded = payload.left(8);
    while (padded.size() < 8) {
        padded.append('\0');
    }
    for (int i = 0; i < 8; ++i) {
        frame[8 + i] = padded.at(i);
    }
    frame[16] = static_cast<char>(0x7A);
    return frame;
}

QList<CanFrame> SerialCanFrameCodec::parse(QByteArray &buffer, quint64 &sequenceCounter)
{
    QList<CanFrame> frames;

    while (!buffer.isEmpty()) {
        const int start = buffer.indexOf(static_cast<char>(0xAA));
        if (start < 0) {
            buffer.clear();
            break;
        }

        if (start > 0) {
            buffer.remove(0, start);
        }

        if (buffer.size() < 17) {
            break;
        }

        if (static_cast<quint8>(buffer.at(16)) != 0x7A) {
            buffer.remove(0, 1);
            continue;
        }

        const quint8 dlc = static_cast<quint8>(buffer.at(3));
        QByteArray payload = buffer.mid(8, qMin<int>(dlc, 8));

        CanFrame frame;
        frame.canId =
            (static_cast<quint8>(buffer.at(6)) << 8)
            | static_cast<quint8>(buffer.at(7));
        frame.data = payload;
        frame.sequence = ++sequenceCounter;
        frames.append(frame);

        buffer.remove(0, 17);
    }

    return frames;
}

