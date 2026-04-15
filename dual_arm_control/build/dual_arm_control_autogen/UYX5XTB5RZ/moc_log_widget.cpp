/****************************************************************************
** Meta object code from reading C++ file 'log_widget.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../ui/log_widget.h"
#include <QtGui/qtextcursor.h>
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'log_widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_dac__LogWidget_t {
    const uint offsetsAndSize[14];
    char stringdata0[60];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_dac__LogWidget_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_dac__LogWidget_t qt_meta_stringdata_dac__LogWidget = {
    {
QT_MOC_LITERAL(0, 14), // "dac::LogWidget"
QT_MOC_LITERAL(15, 9), // "appendLog"
QT_MOC_LITERAL(25, 0), // ""
QT_MOC_LITERAL(26, 13), // "dac::LogLevel"
QT_MOC_LITERAL(40, 5), // "level"
QT_MOC_LITERAL(46, 7), // "message"
QT_MOC_LITERAL(54, 5) // "clear"

    },
    "dac::LogWidget\0appendLog\0\0dac::LogLevel\0"
    "level\0message\0clear"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_dac__LogWidget[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    2,   26,    2, 0x0a,    1 /* Public */,
       6,    0,   31,    2, 0x0a,    4 /* Public */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3, QMetaType::QString,    4,    5,
    QMetaType::Void,

       0        // eod
};

void dac::LogWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<LogWidget *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->appendLog((*reinterpret_cast< std::add_pointer_t<dac::LogLevel>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QString>>(_a[2]))); break;
        case 1: _t->clear(); break;
        default: ;
        }
    }
}

const QMetaObject dac::LogWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_dac__LogWidget.offsetsAndSize,
    qt_meta_data_dac__LogWidget,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_dac__LogWidget_t
, QtPrivate::TypeAndForceComplete<LogWidget, std::true_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<dac::LogLevel, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>


>,
    nullptr
} };


const QMetaObject *dac::LogWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *dac::LogWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_dac__LogWidget.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int dac::LogWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
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
QT_WARNING_POP
QT_END_MOC_NAMESPACE
