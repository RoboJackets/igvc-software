#include <qapplication.h>
#include "mainform.h"


int main( int argc, char ** argv )
{
    QApplication a( argc, argv );
    mainForm w;
    w.show();
    a.connect( &a, SIGNAL( lastWindowClosed() ), &w, SLOT( quit() ) );
    return a.exec();
}
