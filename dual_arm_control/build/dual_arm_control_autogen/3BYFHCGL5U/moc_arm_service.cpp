/****************************************************************************
** Meta object code from reading C++ file 'arm_service.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../services/arm_service.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'arm_service.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_dac__ArmService_t {
    const uint offsetsAndSize[22];
    char stringdata0[126];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_dac__ArmService_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_dac__ArmService_t qt_meta_stringdata_dac__ArmService = {
    {
QT_MOC_LITERAL(0, 15), // "dac::ArmService"
QT_MOC_LITERAL(16, 21), // "initializationChanged"
QT_MOC_LITERAL(38, 0), // ""
QT_MOC_LITERAL(39, 11), // "initialized"
QT_MOC_LITERAL(51, 7), // "message"
QT_MOC_LITERAL(59, 15), // "armStateChanged"
QT_MOC_LITERAL(75, 13), // "dac::ArmState"
QT_MOC_LITERAL(89, 5), // "state"
QT_MOC_LITERAL(95, 10), // "logMessage"
QT_MOC_LITERAL(106, 13), // "dac::LogLevel"
QT_MOC_LITERAL(120, 5) // "level"

    },
    "dac::ArmService\0initializationChanged\0"
    "\0initialized\0message\0armStateChanged\0"
    "dac::ArmState\0state\0logMessage\0"
    "dac::LogLevel\0level"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_dac__ArmService[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    2,   32,    2, 0x06,    1 /* Public */,
       5,    1,   37,    2, 0x06,    4 /* Public */,
       8,    2,   40,    2, 0x06,    6 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool, QMetaType::QString,    3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, 0x80000000 | 9, QMetaType::QString,   10,    4,

       0        // eod
};

void dac::ArmService::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ArmService *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->initializationChanged((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 1: _t->armStateChanged((*reinterpret_cast< std::add_pointer_t<dac::ArmState>>(_a[1]))); break;
        case 2: _t->logMessage((*reinterpret_cast< std::add_pointer_t<dac::LogLevel>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
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
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ArmService::*)(bool , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ArmService::initializationChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (ArmService::*)(const dac::ArmState & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ArmService::armStateChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (ArmService::*)(dac::LogLevel , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ArmService::logMessage)) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject dac::ArmService::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_dac__ArmService.offsetsAndSize,
    qt_meta_data_dac__ArmService,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_dac__ArmService_t
, QtPrivate::TypeAndForceComplete<ArmService, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const dac::ArmState &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<dac::LogLevel, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>



>,
    nullptr
} };


const QMetaObject *dac::ArmService::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *dac::ArmService::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_dac__ArmService.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int dac::ArmService::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void dac::ArmService::initializationChanged(bool _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void dac::ArmService::armStateChanged(const dac::ArmState & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void dac::ArmService::logMessage(dac::LogLevel _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
