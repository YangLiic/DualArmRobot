/****************************************************************************
** Meta object code from reading C++ file 'humanoid_arms_sdk_adapter.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../protocols/humanoid_arms_sdk_adapter.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QList>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'humanoid_arms_sdk_adapter.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_dac__HumanoidArmsSdkAdapter_t {
    const uint offsetsAndSize[52];
    char stringdata0[341];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_dac__HumanoidArmsSdkAdapter_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_dac__HumanoidArmsSdkAdapter_t qt_meta_stringdata_dac__HumanoidArmsSdkAdapter = {
    {
QT_MOC_LITERAL(0, 27), // "dac::HumanoidArmsSdkAdapter"
QT_MOC_LITERAL(28, 21), // "initializationChanged"
QT_MOC_LITERAL(50, 0), // ""
QT_MOC_LITERAL(51, 11), // "initialized"
QT_MOC_LITERAL(63, 7), // "message"
QT_MOC_LITERAL(71, 15), // "armStateUpdated"
QT_MOC_LITERAL(87, 13), // "dac::ArmState"
QT_MOC_LITERAL(101, 5), // "state"
QT_MOC_LITERAL(107, 10), // "logMessage"
QT_MOC_LITERAL(118, 13), // "dac::LogLevel"
QT_MOC_LITERAL(132, 5), // "level"
QT_MOC_LITERAL(138, 13), // "initializeSdk"
QT_MOC_LITERAL(152, 11), // "deviceIndex"
QT_MOC_LITERAL(164, 8), // "canIndex"
QT_MOC_LITERAL(173, 11), // "shutdownSdk"
QT_MOC_LITERAL(185, 12), // "refreshState"
QT_MOC_LITERAL(198, 12), // "dac::ArmSide"
QT_MOC_LITERAL(211, 4), // "side"
QT_MOC_LITERAL(216, 16), // "refreshAllStates"
QT_MOC_LITERAL(233, 21), // "refreshRealtimeStates"
QT_MOC_LITERAL(255, 20), // "moveToJointPositions"
QT_MOC_LITERAL(276, 13), // "QList<double>"
QT_MOC_LITERAL(290, 17), // "jointPositionsRad"
QT_MOC_LITERAL(308, 8), // "velocity"
QT_MOC_LITERAL(317, 8), // "brakeArm"
QT_MOC_LITERAL(326, 14) // "clearArmErrors"

    },
    "dac::HumanoidArmsSdkAdapter\0"
    "initializationChanged\0\0initialized\0"
    "message\0armStateUpdated\0dac::ArmState\0"
    "state\0logMessage\0dac::LogLevel\0level\0"
    "initializeSdk\0deviceIndex\0canIndex\0"
    "shutdownSdk\0refreshState\0dac::ArmSide\0"
    "side\0refreshAllStates\0refreshRealtimeStates\0"
    "moveToJointPositions\0QList<double>\0"
    "jointPositionsRad\0velocity\0brakeArm\0"
    "clearArmErrors"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_dac__HumanoidArmsSdkAdapter[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    2,   80,    2, 0x06,    1 /* Public */,
       5,    1,   85,    2, 0x06,    4 /* Public */,
       8,    2,   88,    2, 0x06,    6 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      11,    2,   93,    2, 0x0a,    9 /* Public */,
      14,    0,   98,    2, 0x0a,   12 /* Public */,
      15,    1,   99,    2, 0x0a,   13 /* Public */,
      18,    0,  102,    2, 0x0a,   15 /* Public */,
      19,    0,  103,    2, 0x0a,   16 /* Public */,
      20,    3,  104,    2, 0x0a,   17 /* Public */,
      24,    1,  111,    2, 0x0a,   21 /* Public */,
      25,    1,  114,    2, 0x0a,   23 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool, QMetaType::QString,    3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, 0x80000000 | 9, QMetaType::QString,   10,    4,

 // slots: parameters
    QMetaType::Void, QMetaType::Int, QMetaType::Int,   12,   13,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 16,   17,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 16, 0x80000000 | 21, QMetaType::Double,   17,   22,   23,
    QMetaType::Void, 0x80000000 | 16,   17,
    QMetaType::Void, 0x80000000 | 16,   17,

       0        // eod
};

void dac::HumanoidArmsSdkAdapter::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<HumanoidArmsSdkAdapter *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->initializationChanged((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 1: _t->armStateUpdated((*reinterpret_cast< std::add_pointer_t<dac::ArmState>>(_a[1]))); break;
        case 2: _t->logMessage((*reinterpret_cast< std::add_pointer_t<dac::LogLevel>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 3: _t->initializeSdk((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[2]))); break;
        case 4: _t->shutdownSdk(); break;
        case 5: _t->refreshState((*reinterpret_cast< std::add_pointer_t<dac::ArmSide>>(_a[1]))); break;
        case 6: _t->refreshAllStates(); break;
        case 7: _t->refreshRealtimeStates(); break;
        case 8: _t->moveToJointPositions((*reinterpret_cast< std::add_pointer_t<dac::ArmSide>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QList<double>>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[3]))); break;
        case 9: _t->brakeArm((*reinterpret_cast< std::add_pointer_t<dac::ArmSide>>(_a[1]))); break;
        case 10: _t->clearArmErrors((*reinterpret_cast< std::add_pointer_t<dac::ArmSide>>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
        case 1:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< dac::ArmState >(); break;
            }
            break;
        case 5:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< dac::ArmSide >(); break;
            }
            break;
        case 8:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 1:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QList<double> >(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< dac::ArmSide >(); break;
            }
            break;
        case 9:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< dac::ArmSide >(); break;
            }
            break;
        case 10:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< dac::ArmSide >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (HumanoidArmsSdkAdapter::*)(bool , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&HumanoidArmsSdkAdapter::initializationChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (HumanoidArmsSdkAdapter::*)(const dac::ArmState & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&HumanoidArmsSdkAdapter::armStateUpdated)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (HumanoidArmsSdkAdapter::*)(dac::LogLevel , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&HumanoidArmsSdkAdapter::logMessage)) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject dac::HumanoidArmsSdkAdapter::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_dac__HumanoidArmsSdkAdapter.offsetsAndSize,
    qt_meta_data_dac__HumanoidArmsSdkAdapter,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_dac__HumanoidArmsSdkAdapter_t
, QtPrivate::TypeAndForceComplete<HumanoidArmsSdkAdapter, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const dac::ArmState &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<dac::LogLevel, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<dac::ArmSide, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<dac::ArmSide, std::false_type>, QtPrivate::TypeAndForceComplete<const QVector<double> &, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<dac::ArmSide, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<dac::ArmSide, std::false_type>


>,
    nullptr
} };


const QMetaObject *dac::HumanoidArmsSdkAdapter::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *dac::HumanoidArmsSdkAdapter::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_dac__HumanoidArmsSdkAdapter.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int dac::HumanoidArmsSdkAdapter::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    }
    return _id;
}

// SIGNAL 0
void dac::HumanoidArmsSdkAdapter::initializationChanged(bool _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void dac::HumanoidArmsSdkAdapter::armStateUpdated(const dac::ArmState & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void dac::HumanoidArmsSdkAdapter::logMessage(dac::LogLevel _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
