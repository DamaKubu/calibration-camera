#include "intrinsic_app.h"

#include <iostream>

int main(int argc, char** argv) {
    intrin::AppConfig cfg;
    try {
        if (!intrin::parseArgs(argc, argv, cfg)) return 0;
        return intrin::run(cfg);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        intrin::printUsage();
        return 2;
    }
}
