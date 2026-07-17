#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include "Foc.h"
#include "Touch.h"
#include "DRV8313.h"

class Application
{
public:
    Application();

    void init();

    void run();

private:
    DRV8313 drv_;
    Foc foc_;
    Touch touch_;
};

#endif