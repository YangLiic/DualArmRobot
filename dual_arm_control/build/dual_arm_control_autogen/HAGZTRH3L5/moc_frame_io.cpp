/****************************************************************************
** Meta object code from reading C++ file 'frame_io.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../communication/frame_io.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'frame_io.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_dac__FrameIO_t {
    const uint offsetsAndSize[14];
    char stringdata0[66];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_dac__FrameIO_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_dac__FrameIO_t qt_meta_stringdata_dac__FrameIO = {
    {
QT_MOC_LITERAL(0, 12), // "dac::FrameIO"
QT_MOC_LITERAL(13, 13), // "frameReceived"
QT_MOC_LITERAL(27, 0), // ""
QT_MOC_LITERAL(28, 13), // "dac::CanFrame"
QT_MOC_LITERAL(42, 5), // "frame"
QT_MOC_LITERAL(48, 13), // "errorOccurred"
QT_MOC_LITERAL(62, 3) // "msg"

    },
    "dac::FrameIO\0frameReceived\0\0dac::CanFrame\0"
    "frame\0errorOccurred\0msg"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_dac__FrameIO[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   26,    2, 0x06,    1 /* Public */,
       5,    1,   29,    2, 0x06,    3 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::QString,    6,

       0        // eod
};

void dac::FrameIO::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<FrameIO *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->frameReceived((*reinterpret_cast< std::add_pointer_t<dac::CanFrame>>(_a[1]))); break;
        case 1: _t->errorOccurred((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (FrameIO::*)(const dac::CanFrame & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&FrameIO::frameReceived)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (FrameIO::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&FrameIO::errorOccurred)) {
                *result = 1;
                return;
            }
        }
    }
}

const QMetaObject dac::FrameIO::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_dac__FrameIO.offsetsAndSize,
    qt_meta_data_dac__FrameIO,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_dac__FrameIO_t
, QtPrivate::TypeAndForceComplete<FrameIO, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const dac::CanFrame &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>



>,
    nullptr
} };


const QMetaObject *dac::FrameIO::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *dac::FrameIO::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_dac__FrameIO.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int dac::FrameIO::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void dac::FrameIO::frameReceived(const dac::CanFrame & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void dac::FrameIO::errorOccurred(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
