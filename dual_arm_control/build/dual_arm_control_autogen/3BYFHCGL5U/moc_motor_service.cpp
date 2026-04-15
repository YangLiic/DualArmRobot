/****************************************************************************
** Meta object code from reading C++ file 'motor_service.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../services/motor_service.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'motor_service.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_dac__MotorService_t {
    const uint offsetsAndSize[40];
    char stringdata0[219];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_dac__MotorService_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_dac__MotorService_t qt_meta_stringdata_dac__MotorService = {
    {
QT_MOC_LITERAL(0, 17), // "dac::MotorService"
QT_MOC_LITERAL(18, 17), // "motorStateChanged"
QT_MOC_LITERAL(36, 0), // ""
QT_MOC_LITERAL(37, 8), // "uint32_t"
QT_MOC_LITERAL(46, 6), // "nodeId"
QT_MOC_LITERAL(53, 15), // "dac::MotorState"
QT_MOC_LITERAL(69, 5), // "state"
QT_MOC_LITERAL(75, 13), // "torqueUpdated"
QT_MOC_LITERAL(89, 7), // "int16_t"
QT_MOC_LITERAL(97, 14), // "torquePermille"
QT_MOC_LITERAL(112, 17), // "collisionDetected"
QT_MOC_LITERAL(130, 12), // "motorEnabled"
QT_MOC_LITERAL(143, 2), // "ok"
QT_MOC_LITERAL(146, 13), // "motorDisabled"
QT_MOC_LITERAL(160, 13), // "faultOccurred"
QT_MOC_LITERAL(174, 9), // "faultText"
QT_MOC_LITERAL(184, 10), // "logMessage"
QT_MOC_LITERAL(195, 13), // "dac::LogLevel"
QT_MOC_LITERAL(209, 5), // "level"
QT_MOC_LITERAL(215, 3) // "msg"

    },
    "dac::MotorService\0motorStateChanged\0"
    "\0uint32_t\0nodeId\0dac::MotorState\0state\0"
    "torqueUpdated\0int16_t\0torquePermille\0"
    "collisionDetected\0motorEnabled\0ok\0"
    "motorDisabled\0faultOccurred\0faultText\0"
    "logMessage\0dac::LogLevel\0level\0msg"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_dac__MotorService[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       7,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    2,   56,    2, 0x06,    1 /* Public */,
       7,    2,   61,    2, 0x06,    4 /* Public */,
      10,    2,   66,    2, 0x06,    7 /* Public */,
      11,    2,   71,    2, 0x06,   10 /* Public */,
      13,    1,   76,    2, 0x06,   13 /* Public */,
      14,    2,   79,    2, 0x06,   15 /* Public */,
      16,    2,   84,    2, 0x06,   18 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5,    4,    6,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 8,    4,    9,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 8,    4,    9,
    QMetaType::Void, 0x80000000 | 3, QMetaType::Bool,    4,   12,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3, QMetaType::QString,    4,   15,
    QMetaType::Void, 0x80000000 | 17, QMetaType::QString,   18,   19,

       0        // eod
};

void dac::MotorService::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MotorService *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->motorStateChanged((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<dac::MotorState>>(_a[2]))); break;
        case 1: _t->torqueUpdated((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int16_t>>(_a[2]))); break;
        case 2: _t->collisionDetected((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int16_t>>(_a[2]))); break;
        case 3: _t->motorEnabled((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[2]))); break;
        case 4: _t->motorDisabled((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1]))); break;
        case 5: _t->faultOccurred((*reinterpret_cast< std::add_pointer_t<uint32_t>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 6: _t->logMessage((*reinterpret_cast< std::add_pointer_t<dac::LogLevel>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (MotorService::*)(uint32_t , const dac::MotorState & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorService::motorStateChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (MotorService::*)(uint32_t , int16_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorService::torqueUpdated)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (MotorService::*)(uint32_t , int16_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorService::collisionDetected)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (MotorService::*)(uint32_t , bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorService::motorEnabled)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (MotorService::*)(uint32_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorService::motorDisabled)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (MotorService::*)(uint32_t , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorService::faultOccurred)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (MotorService::*)(dac::LogLevel , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MotorService::logMessage)) {
                *result = 6;
                return;
            }
        }
    }
}

const QMetaObject dac::MotorService::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_dac__MotorService.offsetsAndSize,
    qt_meta_data_dac__MotorService,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_dac__MotorService_t
, QtPrivate::TypeAndForceComplete<MotorService, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<const dac::MotorState &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<int16_t, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<int16_t, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<uint32_t, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<dac::LogLevel, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>



>,
    nullptr
} };


const QMetaObject *dac::MotorService::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *dac::MotorService::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_dac__MotorService.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int dac::MotorService::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void dac::MotorService::motorStateChanged(uint32_t _t1, const dac::MotorState & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void dac::MotorService::torqueUpdated(uint32_t _t1, int16_t _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void dac::MotorService::collisionDetected(uint32_t _t1, int16_t _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void dac::MotorService::motorEnabled(uint32_t _t1, bool _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void dac::MotorService::motorDisabled(uint32_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void dac::MotorService::faultOccurred(uint32_t _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void dac::MotorService::logMessage(dac::LogLevel _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
