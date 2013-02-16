#include <QEI.h>

Serial pc(USBTX, USBRX);
//Use X4 encoding.
//QEI wheel(p29, p30, NC, 624, QEI::X4_ENCODING);
//Use X2 encoding by default.
QEI wheel (p29, p30, NC, 624);

int main() {

    while(1){
        wait(0.1);
        pc.printf("Pulses is: %i\n", wheel.getPulses());
    }

}
