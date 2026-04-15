/****************************************************************************
** Meta object code from reading C++ file 'communication_manager.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../communication/communication_manager.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'communication_manager.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_dac__CommunicationManager_t {
    const uint offsetsAndSize[36];
    char stringdata0[204];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_dac__CommunicationManager_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_dac__CommunicationManager_t qt_meta_stringdata_dac__CommunicationManager = {
    {
QT_MOC_LITERAL(0, 25), // "dac::CommunicationManager"
QT_MOC_LITERAL(26, 15), // "busStateChanged"
QT_MOC_LITERAL(42, 0), // ""
QT_MOC_LITERAL(43, 4), // "open"
QT_MOC_LITERAL(48, 3), // "msg"
QT_MOC_LITERAL(52, 16), // "rawFrameReceived"
QT_MOC_LITERAL(69, 13), // "dac::CanFrame"
QT_MOC_LITERAL(83, 5), // "frame"
QT_MOC_LITERAL(89, 10), // "logMessage"
QT_MOC_LITERAL(100, 13), // "dac::LogLevel"
QT_MOC_LITERAL(114, 5), // "level"
QT_MOC_LITERAL(120, 11), // "onBusOpened"
QT_MOC_LITERAL(132, 2), // "ok"
QT_MOC_LITERAL(135, 18), // "onRequestCompleted"
QT_MOC_LITERAL(154, 15), // "dac::CommResult"
QT_MOC_LITERAL(170, 6), // "result"
QT_MOC_LITERAL(177, 10), // "onRawFrame"
QT_MOC_LITERAL(188, 15) // "onPollTimerTick"

    },
    "dac::CommunicationManager\0busStateChanged\0"
    "\0open\0msg\0rawFrameReceived\0dac::CanFrame\0"
    "frame\0logMessage\0dac::LogLevel\0level\0"
    "onBusOpened\0ok\0onRequestCompleted\0"
    "dac::CommResult\0result\0onRawFrame\0"
    "onPollTimerTick"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_dac__CommunicationManager[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    2,   56,    2, 0x06,    1 /* Public */,
       5,    1,   61,    2, 0x06,    4 /* Public */,
       8,    2,   64,    2, 0x06,    6 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      11,    2,   69,    2, 0x08,    9 /* Private */,
      13,    1,   74,    2, 0x08,   12 /* Private */,
      16,    1,   77,    2, 0x08,   14 /* Private */,
      17,    0,   80,    2, 0x08,   16 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool, QMetaType::QString,    3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, 0x80000000 | 9, QMetaType::QString,   10,    4,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool, QMetaType::QString,   12,    4,
    QMetaType::Void, 0x80000000 | 14,   15,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void,

       0        // eod
};

void dac::CommunicationManager::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<CommunicationManager *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->busStateChanged((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 1: _t->rawFrameReceived((*reinterpret_cast< std::add_pointer_t<dac::CanFrame>>(_a[1]))); break;
        case 2: _t->logMessage((*reinterpret_cast< std::add_pointer_t<dac::LogLevel>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 3: _t->onBusOpened((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 4: _t->onRequestCompleted((*reinterpret_cast< std::add_pointer_t<dac::CommResult>>(_a[1]))); break;
        case 5: _t->onRawFrame((*reinterpret_cast< std::add_pointer_t<dac::CanFrame>>(_a[1]))); break;
        case 6: _t->onPollTimerTick(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (CommunicationManager::*)(bool , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CommunicationManager::busStateChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (CommunicationManager::*)(const dac::CanFrame & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CommunicationManager::rawFrameReceived)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (CommunicationManager::*)(dac::LogLevel , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CommunicationManager::logMessage)) {
                *result = 2;
                return;
            }
        }
    }
}

const QMetaObject dac::CommunicationManager::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_dac__CommunicationManager.offsetsAndSize,
    qt_meta_data_dac__CommunicationManager,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_dac__CommunicationManager_t
, QtPrivate::TypeAndForceComplete<CommunicationManager, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const dac::CanFrame &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<dac::LogLevel, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const dac::CommResult &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const dac::CanFrame &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>


>,
    nullptr
} };


const QMetaObject *dac::CommunicationManager::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *dac::CommunicationManager::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_dac__CommunicationManager.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int dac::CommunicationManager::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void dac::CommunicationManager::busStateChanged(bool _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void dac::CommunicationManager::rawFrameReceived(const dac::CanFrame & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void dac::CommunicationManager::logMessage(dac::LogLevel _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
