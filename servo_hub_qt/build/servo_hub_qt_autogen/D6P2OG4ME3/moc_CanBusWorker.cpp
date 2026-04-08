/****************************************************************************
** Meta object code from reading C++ file 'CanBusWorker.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../src/communication/CanBusWorker.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'CanBusWorker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CanBusWorker_t {
    const uint offsetsAndSize[20];
    char stringdata0[120];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_CanBusWorker_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_CanBusWorker_t qt_meta_stringdata_CanBusWorker = {
    {
QT_MOC_LITERAL(0, 12), // "CanBusWorker"
QT_MOC_LITERAL(13, 10), // "connectBus"
QT_MOC_LITERAL(24, 0), // ""
QT_MOC_LITERAL(25, 13), // "disconnectBus"
QT_MOC_LITERAL(39, 13), // "submitRequest"
QT_MOC_LITERAL(53, 10), // "BusRequest"
QT_MOC_LITERAL(64, 7), // "request"
QT_MOC_LITERAL(72, 11), // "onReadyRead"
QT_MOC_LITERAL(84, 16), // "onRequestTimeout"
QT_MOC_LITERAL(101, 18) // "processNextRequest"

    },
    "CanBusWorker\0connectBus\0\0disconnectBus\0"
    "submitRequest\0BusRequest\0request\0"
    "onReadyRead\0onRequestTimeout\0"
    "processNextRequest"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CanBusWorker[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   50,    2, 0x0a,    1 /* Public */,
       3,    0,   51,    2, 0x0a,    2 /* Public */,
       4,    1,   52,    2, 0x0a,    3 /* Public */,
       7,    0,   55,    2, 0x08,    5 /* Private */,
       8,    0,   56,    2, 0x08,    6 /* Private */,
       9,    0,   57,    2, 0x08,    7 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void CanBusWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<CanBusWorker *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->connectBus(); break;
        case 1: _t->disconnectBus(); break;
        case 2: _t->submitRequest((*reinterpret_cast< std::add_pointer_t<BusRequest>>(_a[1]))); break;
        case 3: _t->onReadyRead(); break;
        case 4: _t->onRequestTimeout(); break;
        case 5: _t->processNextRequest(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
        case 2:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< BusRequest >(); break;
            }
            break;
        }
    }
}

const QMetaObject CanBusWorker::staticMetaObject = { {
    QMetaObject::SuperData::link<AbstractBusWorker::staticMetaObject>(),
    qt_meta_stringdata_CanBusWorker.offsetsAndSize,
    qt_meta_data_CanBusWorker,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_CanBusWorker_t
, QtPrivate::TypeAndForceComplete<CanBusWorker, std::true_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const BusRequest &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>


>,
    nullptr
} };


const QMetaObject *CanBusWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CanBusWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CanBusWorker.stringdata0))
        return static_cast<void*>(this);
    return AbstractBusWorker::qt_metacast(_clname);
}

int CanBusWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = AbstractBusWorker::qt_metacall(_c, _id, _a);
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
QT_WARNING_POP
QT_END_MOC_NAMESPACE
