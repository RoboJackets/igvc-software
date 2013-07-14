// =============================================================================
// Delegate Class for .NET Style Events
// by Clint Caywood
// February 18, 2010
// This class should not be used directly. Use LISTENER instead.
// =============================================================================

#ifndef DELEGATE_HPP
#define DELEGATE_HPP

#define LISTENER(thisType, handler, type)\
    class __L##handler##__ : public Delegate< type >\
    {\
        public:\
            __L##handler##__ ( thisType * obj )\
            : _obj(obj) {}\
            inline void operator()( type param )\
            {\
                _obj-> handler (param);\
            }\
            thisType * _obj;\
    };\
    __L##handler##__ L##handler;

template <typename T>
class Delegate
{
    public:
        virtual void operator()(T param) = 0;
};
#endif
