/****************************************************************************
** Meta object code from reading C++ file 'AbstractBusWorker.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../src/communication/AbstractBusWorker.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'AbstractBusWorker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_AbstractBusWorker_t {
    const uint offsetsAndSize[30];
    char stringdata0[167];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_AbstractBusWorker_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_AbstractBusWorker_t qt_meta_stringdata_AbstractBusWorker = {
    {
QT_MOC_LITERAL(0, 17), // "AbstractBusWorker"
QT_MOC_LITERAL(18, 16), // "requestCompleted"
QT_MOC_LITERAL(35, 0), // ""
QT_MOC_LITERAL(36, 9), // "BusResult"
QT_MOC_LITERAL(46, 6), // "result"
QT_MOC_LITERAL(53, 20), // "busConnectionChanged"
QT_MOC_LITERAL(74, 5), // "busId"
QT_MOC_LITERAL(80, 9), // "connected"
QT_MOC_LITERAL(90, 7), // "message"
QT_MOC_LITERAL(98, 10), // "logMessage"
QT_MOC_LITERAL(109, 10), // "connectBus"
QT_MOC_LITERAL(120, 13), // "disconnectBus"
QT_MOC_LITERAL(134, 13), // "submitRequest"
QT_MOC_LITERAL(148, 10), // "BusRequest"
QT_MOC_LITERAL(159, 7) // "request"

    },
    "AbstractBusWorker\0requestCompleted\0\0"
    "BusResult\0result\0busConnectionChanged\0"
    "busId\0connected\0message\0logMessage\0"
    "connectBus\0disconnectBus\0submitRequest\0"
    "BusRequest\0request"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_AbstractBusWorker[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   50,    2, 0x06,    1 /* Public */,
       5,    3,   53,    2, 0x06,    3 /* Public */,
       9,    1,   60,    2, 0x06,    7 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      10,    0,   63,    2, 0x0a,    9 /* Public */,
      11,    0,   64,    2, 0x0a,   10 /* Public */,
      12,    1,   65,    2, 0x0a,   11 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::QString, QMetaType::Bool, QMetaType::QString,    6,    7,    8,
    QMetaType::Void, QMetaType::QString,    8,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 13,   14,

       0        // eod
};

void AbstractBusWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<AbstractBusWorker *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->requestCompleted((*reinterpret_cast< std::add_pointer_t<BusResult>>(_a[1]))); break;
        case 1: _t->busConnectionChanged((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[3]))); break;
        case 2: _t->logMessage((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 3: _t->connectBus(); break;
        case 4: _t->disconnectBus(); break;
        case 5: _t->submitRequest((*reinterpret_cast< std::add_pointer_t<BusRequest>>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
        case 0:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< BusResult >(); break;
            }
            break;
        case 5:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< BusRequest >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (AbstractBusWorker::*)(const BusResult & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AbstractBusWorker::requestCompleted)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (AbstractBusWorker::*)(const QString & , bool , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AbstractBusWorker::busConnectionChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (AbstractBusWorker::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&AbstractBusWorker::logMessage)) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject AbstractBusWorker::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_AbstractBusWorker.offsetsAndSize,
    qt_meta_data_AbstractBusWorker,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_AbstractBusWorker_t
, QtPrivate::TypeAndForceComplete<AbstractBusWorker, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const BusResult &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const BusRequest &, std::false_type>


>,
    nullptr
} };


const QMetaObject *AbstractBusWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *AbstractBusWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_AbstractBusWorker.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int AbstractBusWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void AbstractBusWorker::requestCompleted(const BusResult & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void AbstractBusWorker::busConnectionChanged(const QString & _t1, bool _t2, const QString & _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void AbstractBusWorker::logMessage(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
