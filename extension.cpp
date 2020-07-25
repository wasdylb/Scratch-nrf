#include "pxt.h"
#include "ScratchMoreService.h"

#define NOTIFY_PERIOD 101

// enum Slot {
//     //% block="slot0"
//     SLOT0 = 0,
//     //% block="slot1"
//     SLOT1 = 1,
//     //% block="slot2"
//     SLOT2 = 2,
//     //% block="slot3"
//     SLOT3 = 3,
// };

//% color=#FF9900 weight=95 icon="\uf1b0"
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
    // void startScratchMoreService(Action handler) {
    //     if (NULL != _pService) return;

    //     _pService = new ScratchMoreService(uBit);
    //     _handler = handler;
    //     pxt::incr(_handler);
    //     create_fiber(notifyScratch);
    // }

    void startMbitMoreService() {
        if (NULL != _pService) return;

        _pService = new MbitMoreService(uBit);
        create_fiber(notifyScratch);
    }

    // /**
    // * Set slot value.
    // */
    // //%
    // void setScratchMoreSlot(Slot slot, int value) {
    //     if (NULL == _pService) return;

    //     _pService->setSlot((int)slot, value);
    // }

    // /**
    //  * Get slot value. 
    //  */
    // //%
    // int getScratchMoreSlot(Slot slot) {
    //     if (NULL == _pService) return 0;

    //     return _pService->getSlot((int)slot);
    // }    
}
