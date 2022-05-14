
/**
 * Custom blocks
 */
//% weight=100 color=#0fbc11 icon="\uf1eb" block="IR"
namespace IR {
    let state: number;
    let data1: number;
    let irstate: number;
    let irData: number = -1;

    //% shim=DFRobotIR::irCode
    function irCode(): number {
        return 0;
    }

    /**
     * TODO: describe your function here
     * @param n describe parameter here, eg: 5
     * @param s describe parameter here, eg: "Hello"
     * @param e describe parameter here
     */
    //% weight=60
    //% block="read IR key value"
    export function IR_read(): number {
        pins.setPull(DigitalPin.P16, PinPullMode.PullUp)
        return valuotokeyConversion();
    }

    /**
     * TODO: describe your function here
     * @param value describe value here, eg: 5
     */
    //% weight=50
    //% block="on IR received"
    //% draggableParameters
    export function IR_callbackUser(cb: (message: number) => void) {
        pins.setPull(DigitalPin.P16, PinPullMode.PullUp)
        state = 1;
        control.onEvent(11, 22, function () {
            cb(data1)

        })
    }

    basic.forever(() => {
        if (state == 1) {
            irstate = irCode();
            if (irstate != 0) {
                data1 = irstate & 0xff;
                control.raiseEvent(11, 22)
            }
        }

        basic.pause(50);
    })

    function valuotokeyConversion(): number {
        //serial.writeValue("x", irCode() )
        let data = irCode();
        if (data == 0) {
        } else {
            irData = data & 0xff;
        }
        return irData;
    }
}
