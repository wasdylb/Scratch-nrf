#include "pxt.h"
#include "ScratchMoreService.h"

#define NOTIFY_PERIOD 50

//% color=#FF00FF weight=95 icon="\uf1b0"
namespace ScratchMore {
    ScratchMoreService* _pService = NULL;

    void notifyScratch() {
        while (NULL != _pService) {
            // notyfy data to Scratch
            _pService->notify();
            fiber_sleep(NOTIFY_PERIOD);
        }
    }

    /**
    * Starts a Scratch extension service.
    * The handler can call ``setscratchMoreSlot`` to send any data to Scratch.
    */
    //%
    void startScratchMoreService() {
        if (NULL != _pService) return;

        _pService = new ScratchMoreService(uBit);
        create_fiber(notifyScratch);
    }
 
}
