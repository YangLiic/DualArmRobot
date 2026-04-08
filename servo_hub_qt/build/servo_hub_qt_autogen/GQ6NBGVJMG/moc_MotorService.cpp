/****************************************************************************
** Meta object code from reading C++ file 'MotorService.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../src/services/MotorService.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QList>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MotorService.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MotorService_t {
    const uint offsetsAndSize[82];
    char stringdata0[514];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_MotorService_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_MotorService_t qt_meta_stringdata_MotorService = {
    {
QT_MOC_LITERAL(0, 12), // "MotorService"
QT_MOC_LITERAL(13, 10), // "logMessage"
QT_MOC_LITERAL(24, 0), // ""
QT_MOC_LITERAL(25, 7), // "message"
QT_MOC_LITERAL(33, 20), // "busConnectionChanged"
QT_MOC_LITERAL(54, 9), // "connected"
QT_MOC_LITERAL(64, 22), // "configuredNodesChanged"
QT_MOC_LITERAL(87, 14), // "QList<quint32>"
QT_MOC_LITERAL(102, 7), // "nodeIds"
QT_MOC_LITERAL(110, 17), // "motorStateChanged"
QT_MOC_LITERAL(128, 10), // "MotorState"
QT_MOC_LITERAL(139, 5), // "state"
QT_MOC_LITERAL(145, 10), // "connectBus"
QT_MOC_LITERAL(156, 13), // "disconnectBus"
QT_MOC_LITERAL(170, 19), // "scanConfiguredNodes"
QT_MOC_LITERAL(190, 15), // "startMonitoring"
QT_MOC_LITERAL(206, 14), // "stopMonitoring"
QT_MOC_LITERAL(221, 14), // "setFocusedNode"
QT_MOC_LITERAL(236, 6), // "nodeId"
QT_MOC_LITERAL(243, 11), // "enableMotor"
QT_MOC_LITERAL(255, 12), // "positionMode"
QT_MOC_LITERAL(268, 12), // "disableMotor"
QT_MOC_LITERAL(281, 14), // "quickStopMotor"
QT_MOC_LITERAL(296, 15), // "faultResetMotor"
QT_MOC_LITERAL(312, 11), // "setVelocity"
QT_MOC_LITERAL(324, 3), // "rpm"
QT_MOC_LITERAL(328, 12), // "movePosition"
QT_MOC_LITERAL(341, 7), // "degrees"
QT_MOC_LITERAL(349, 8), // "absolute"
QT_MOC_LITERAL(358, 15), // "profileVelocity"
QT_MOC_LITERAL(374, 12), // "acceleration"
QT_MOC_LITERAL(387, 12), // "deceleration"
QT_MOC_LITERAL(400, 9), // "enableAll"
QT_MOC_LITERAL(410, 10), // "disableAll"
QT_MOC_LITERAL(421, 12), // "quickStopAll"
QT_MOC_LITERAL(434, 13), // "faultResetAll"
QT_MOC_LITERAL(448, 15), // "handleBusResult"
QT_MOC_LITERAL(464, 9), // "BusResult"
QT_MOC_LITERAL(474, 6), // "result"
QT_MOC_LITERAL(481, 26), // "handleBusConnectionChanged"
QT_MOC_LITERAL(508, 5) // "busId"

    },
    "MotorService\0logMessage\0\0message\0"
    "busConnectionChanged\0connected\0"
    "configuredNodesChanged\0QList<quint32>\0"
    "nodeIds\0motorStateChanged\0MotorState\0"
    "state\0connectBus\0disconnectBus\0"
    "scanConfiguredNodes\0startMonitoring\0"
    "stopMonitoring\0setFocusedNode\0nodeId\0"
    "enableMotor\0positionMode\0disableMotor\0"
    "quickStopMotor\0faultResetMotor\0"
    "setVelocity\0rpm\0movePosition\0degrees\0"
    "absolute\0profileVelocity\0acceleration\0"
    "deceleration\0enableAll\0disableAll\0"
    "quickStopAll\0faultResetAll\0handleBusResult\0"
    "BusResult\0result\0handleBusConnectionChanged\0"
    "busId"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MotorService[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      22,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,  146,    2, 0x06,    1 /* Public */,
       4,    2,  149,    2, 0x06,    3 /* Public */,
       6,    1,  154,    2, 0x06,    6 /* Public */,
       9,    1,  157,    2, 0x06,    8 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      12,    0,  160,    2, 0x0a,   10 /* Public */,
      13,    0,  161,    2, 0x0a,   11 /* Public */,
      14,    0,  162,    2, 0x0a,   12 /* Public */,
      15,    0,  163,    2, 0x0a,   13 /* Public */,
      16,    0,  164,    2, 0x0a,   14 /* Public */,
      17,    1,  165,    2, 0x0a,   15 /* Public */,
      19,    2,  168,    2, 0x0a,   17 /* Public */,
      21,    1,  173,    2, 0x0a,   20 /* Public */,
      22,    1,  176,    2, 0x0a,   22 /* Public */,
      23,    1,  179,    2, 0x0a,   24 /* Public */,
      24,    2,  182,    2, 0x0a,   26 /* Public */,
      26,    6,  187,    2, 0x0a,   29 /* Public */,
      32,    1,  200,    2, 0x0a,   36 /* Public */,
      33,    0,  203,    2, 0x0a,   38 /* Public */,
      34,    0,  204,    2, 0x0a,   39 /* Public */,
      35,    0,  205,    2, 0x0a,   40 /* Public */,
      36,    1,  206,    2, 0x08,   41 /* Private */,
      39,    3,  209,    2, 0x08,   43 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::Bool, QMetaType::QString,    5,    3,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void, 0x80000000 | 10,   11,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::UInt,   18,
    QMetaType::Void, QMetaType::UInt, QMetaType::Bool,   18,   20,
    QMetaType::Void, QMetaType::UInt,   18,
    QMetaType::Void, QMetaType::UInt,   18,
    QMetaType::Void, QMetaType::UInt,   18,
    QMetaType::Void, QMetaType::UInt, QMetaType::Int,   18,   25,
    QMetaType::Void, QMetaType::UInt, QMetaType::Double, QMetaType::Bool, QMetaType::UInt, QMetaType::UInt, QMetaType::UInt,   18,   27,   28,   29,   30,   31,
    QMetaType::Void, QMetaType::Bool,   20,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 37,   38,
    QMetaType::Void, QMetaType::QString, QMetaType::Bool, QMetaType::QString,   40,    5,    3,

       0        // eod
};

void MotorService::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MotorService *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->logMessage((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 1: _t->busConnectionChanged((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 2: _t->configuredNodesChanged((*reinterpret_cast< std::add_pointer_t<QList<quint32>>>(_a[1]))); break;
        case 3: _t->motorStateChanged((*reinterpret_cast< std::add_pointer_t<MotorState>>(_a[1]))); break;
        case 4: _t->connectBus(); break;
        case 5: _t->disconnectBus(); break;
        case 6: _t->scanConfiguredNodes(); break;
        case 7: _t->startMonitoring(); break;
        case 8: _t->stopMonitoring(); break;
        case 9: _t->setFocusedNode((*reinterpret_cast< std::add_pointer_t<quint32>>(_a[1]))); break;
        case 10: _t->enableMotor((*reinterpret_cast< std::add_pointer_t<quint32>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[2]))); break;
        case 11: _t->disableMotor((*reinterpret_cast< std::add_pointer_t<quint32>>(_a[1]))); break;
        case 12: _t->quickStopMotor((*reinterpret_cast< std::add_pointer_t<quint32>>(_a[1]))); break;
        case 13: _t->faultResetMotor((*reinterpret_cast< std::add_pointer_t<quint32>>(_a[1]))); break;
        case 14: _t->setVelocity((*reinterpret_cast< std::add_pointer_t<quint32>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<qint32>>(_a[2]))); break;
        case 15: _t->movePosition((*reinterpret_cast< std::add_pointer_t<quint32>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<quint32>>(_a[4])),(*reinterpret_cast< std::add_pointer_t<quint32>>(_a[5])),(*reinterpret_cast< std::add_pointer_t<quint32>>(_a[6]))); break;
        case 16: _t->enableAll((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 17: _t->disableAll(); break;
        case 18: _t->quickStopAll(); break;
        case 19: _t->faultResetAll(); break;
        case 20: _t->handleBusResult((*reinterpret_cast< std::add_pointer_t<BusResult>>(_a[1]))); break;
        case 21: _t->handleBusConnectionChanged((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[3]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
        case 2:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QList<quint32> >(); break;
            }
            break;
        case 3:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< MotorState >(); break;
            }
            break;
        case 20:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< BusResult >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (MotorService::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorService::logMessage)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (MotorService::*)(bool , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorService::busConnectionChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (MotorService::*)(const QList<quint32> & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorService::configuredNodesChanged)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (MotorService::*)(const MotorState & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorService::motorStateChanged)) {
                *result = 3;
                return;
            }
        }
    }
}

const QMetaObject MotorService::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_MotorService.offsetsAndSize,
    qt_meta_data_MotorService,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_MotorService_t
, QtPrivate::TypeAndForceComplete<MotorService, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QList<quint32> &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const MotorState &, std::false_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<quint32, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<quint32, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<quint32, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<quint32, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<quint32, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<quint32, std::false_type>, QtPrivate::TypeAndForceComplete<qint32, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<quint32, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<quint32, std::false_type>, QtPrivate::TypeAndForceComplete<quint32, std::false_type>, QtPrivate::TypeAndForceComplete<quint32, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const BusResult &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>


>,
    nullptr
} };


const QMetaObject *MotorService::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MotorService::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MotorService.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int MotorService::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 22)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 22;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 22)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 22;
    }
    return _id;
}

// SIGNAL 0
void MotorService::logMessage(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void MotorService::busConnectionChanged(bool _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void MotorService::configuredNodesChanged(const QList<quint32> & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void MotorService::motorStateChanged(const MotorState & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
