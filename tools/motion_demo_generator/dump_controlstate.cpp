// Prints the exact wire JSON of a default-constructed ControlState (one line) so the demo-runner
// python client can patch fields instead of hand-crafting the strict 35-field schema.
#include "RdtJson.h"
#include <iostream>

int main() {
    RDT::NetProtocol::ControlState cs;
    auto res = RDT::safeSerialize(cs);
    if (res.isError()) {
        std::cerr << "serialize failed" << std::endl;
        return 1;
    }
    std::cout << res.value() << std::endl;
    return 0;
}
