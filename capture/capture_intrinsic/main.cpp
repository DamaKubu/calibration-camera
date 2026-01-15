#include "capture_intrinsic_app.h"

#include <iostream>

int main(int argc, char** argv) {
    capintr::AppConfig cfg;
    std::string err;
    if (!capintr::parseArgs(argc, argv, cfg, err)) {
        if (err == "help") {
            capintr::printUsage(std::cout);
            return 0;
        }
        std::cerr << "ERROR: " << err << "\n\n";
        capintr::printUsage(std::cerr);
        return 2;
    }
    return capintr::run(cfg);
}
