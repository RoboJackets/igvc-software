/****************************************************************************
** Meta object code from reading C++ file 'DCam_Config.h'
**
** Created: Sat Sep 19 15:37:43 2009
**      by: The Qt Meta Object Compiler version 61 (Qt 4.5.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "vision/DCam_Config.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'DCam_Config.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 61
#error "This file was generated using the moc from 4.5.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_DCam_Feature[] = {

 // content:
       2,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   12, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors

 // slots: signature, parameters, type, tag, flags
      17,   14,   13,   13, 0x09,
      42,   36,   13,   13, 0x09,
      63,   36,   13,   13, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_DCam_Feature[] = {
    "DCam_Feature\0\0on\0auto_changed(bool)\0"
    "value\0slider0_changed(int)\0"
    "slider1_changed(int)\0"
};

const QMetaObject DCam_Feature::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_DCam_Feature,
      qt_meta_data_DCam_Feature, 0 }
};

const QMetaObject *DCam_Feature::metaObject() const
{
    return &staticMetaObject;
}

void *DCam_Feature::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_DCam_Feature))
        return static_cast<void*>(const_cast< DCam_Feature*>(this));
    return QObject::qt_metacast(_clname);
}

int DCam_Feature::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: auto_changed((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: slider0_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: slider1_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 3;
    }
    return _id;
}
static const uint qt_meta_data_DCam_Config[] = {

 // content:
       2,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors

       0        // eod
};

static const char qt_meta_stringdata_DCam_Config[] = {
    "DCam_Config\0"
};

const QMetaObject DCam_Config::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_DCam_Config,
      qt_meta_data_DCam_Config, 0 }
};

const QMetaObject *DCam_Config::metaObject() const
{
    return &staticMetaObject;
}

void *DCam_Config::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_DCam_Config))
        return static_cast<void*>(const_cast< DCam_Config*>(this));
    return QWidget::qt_metacast(_clname);
}

int DCam_Config::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
QT_END_MOC_NAMESPACE
