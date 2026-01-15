#include "live_motion_app.h"

#include <iostream>

int main(int argc, char** argv) {
    livemotion::AppConfig cfg;
    try {
        if (!livemotion::parseArgs(argc, argv, cfg)) return 0;
        return livemotion::run(cfg);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        livemotion::printUsage(std::cout);
        return 2;
    }
}
