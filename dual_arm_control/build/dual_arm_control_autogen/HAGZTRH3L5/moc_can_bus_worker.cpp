/****************************************************************************
** Meta object code from reading C++ file 'can_bus_worker.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../communication/can_bus_worker.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'can_bus_worker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_dac__CanBusWorker_t {
    const uint offsetsAndSize[42];
    char stringdata0[204];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_dac__CanBusWorker_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_dac__CanBusWorker_t qt_meta_stringdata_dac__CanBusWorker = {
    {
QT_MOC_LITERAL(0, 17), // "dac::CanBusWorker"
QT_MOC_LITERAL(18, 9), // "busOpened"
QT_MOC_LITERAL(28, 0), // ""
QT_MOC_LITERAL(29, 2), // "ok"
QT_MOC_LITERAL(32, 3), // "msg"
QT_MOC_LITERAL(36, 9), // "busClosed"
QT_MOC_LITERAL(46, 16), // "requestCompleted"
QT_MOC_LITERAL(63, 15), // "dac::CommResult"
QT_MOC_LITERAL(79, 6), // "result"
QT_MOC_LITERAL(86, 13), // "frameReceived"
QT_MOC_LITERAL(100, 13), // "dac::CanFrame"
QT_MOC_LITERAL(114, 5), // "frame"
QT_MOC_LITERAL(120, 10), // "logMessage"
QT_MOC_LITERAL(131, 13), // "dac::LogLevel"
QT_MOC_LITERAL(145, 5), // "level"
QT_MOC_LITERAL(151, 5), // "start"
QT_MOC_LITERAL(157, 4), // "stop"
QT_MOC_LITERAL(162, 13), // "submitRequest"
QT_MOC_LITERAL(176, 16), // "dac::CommRequest"
QT_MOC_LITERAL(193, 3), // "req"
QT_MOC_LITERAL(197, 6) // "onTick"

    },
    "dac::CanBusWorker\0busOpened\0\0ok\0msg\0"
    "busClosed\0requestCompleted\0dac::CommResult\0"
    "result\0frameReceived\0dac::CanFrame\0"
    "frame\0logMessage\0dac::LogLevel\0level\0"
    "start\0stop\0submitRequest\0dac::CommRequest\0"
    "req\0onTick"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_dac__CanBusWorker[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    2,   68,    2, 0x06,    1 /* Public */,
       5,    0,   73,    2, 0x06,    4 /* Public */,
       6,    1,   74,    2, 0x06,    5 /* Public */,
       9,    1,   77,    2, 0x06,    7 /* Public */,
      12,    2,   80,    2, 0x06,    9 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      15,    0,   85,    2, 0x0a,   12 /* Public */,
      16,    0,   86,    2, 0x0a,   13 /* Public */,
      17,    1,   87,    2, 0x0a,   14 /* Public */,
      20,    0,   90,    2, 0x08,   16 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool, QMetaType::QString,    3,    4,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, 0x80000000 | 13, QMetaType::QString,   14,    4,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 18,   19,
    QMetaType::Void,

       0        // eod
};

void dac::CanBusWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<CanBusWorker *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->busOpened((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 1: _t->busClosed(); break;
        case 2: _t->requestCompleted((*reinterpret_cast< std::add_pointer_t<dac::CommResult>>(_a[1]))); break;
        case 3: _t->frameReceived((*reinterpret_cast< std::add_pointer_t<dac::CanFrame>>(_a[1]))); break;
        case 4: _t->logMessage((*reinterpret_cast< std::add_pointer_t<dac::LogLevel>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 5: _t->start(); break;
        case 6: _t->stop(); break;
        case 7: _t->submitRequest((*reinterpret_cast< std::add_pointer_t<dac::CommRequest>>(_a[1]))); break;
        case 8: _t->onTick(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (CanBusWorker::*)(bool , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CanBusWorker::busOpened)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (CanBusWorker::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CanBusWorker::busClosed)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (CanBusWorker::*)(const dac::CommResult & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CanBusWorker::requestCompleted)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (CanBusWorker::*)(const dac::CanFrame & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CanBusWorker::frameReceived)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (CanBusWorker::*)(dac::LogLevel , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CanBusWorker::logMessage)) {
                *result = 4;
                return;
            }
        }
    }
}

const QMetaObject dac::CanBusWorker::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_dac__CanBusWorker.offsetsAndSize,
    qt_meta_data_dac__CanBusWorker,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_dac__CanBusWorker_t
, QtPrivate::TypeAndForceComplete<CanBusWorker, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const dac::CommResult &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const dac::CanFrame &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<dac::LogLevel, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const dac::CommRequest &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>


>,
    nullptr
} };


const QMetaObject *dac::CanBusWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *dac::CanBusWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_dac__CanBusWorker.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int dac::CanBusWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 9;
    }
    return _id;
}

// SIGNAL 0
void dac::CanBusWorker::busOpened(bool _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void dac::CanBusWorker::busClosed()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void dac::CanBusWorker::requestCompleted(const dac::CommResult & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void dac::CanBusWorker::frameReceived(const dac::CanFrame & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void dac::CanBusWorker::logMessage(dac::LogLevel _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
