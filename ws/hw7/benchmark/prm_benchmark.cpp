
#include "prm_benchmark.h"
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"
#include <fstream>
#include <chrono>
#include <vector>
#include <utility>
#include <iostream>

using namespace amp;

void run_prm_benchmark(const std::string& ws_name, int ws_num) {
    Problem2D problem;
    if (ws_name == "HW2") {
        if (ws_num == 1) problem = HW2::getWorkspace1();
        else if (ws_num == 2) problem = HW2::getWorkspace2();
        else { std::cerr << "Invalid HW2 workspace number\n"; return; }
    } else if (ws_name == "HW5") {
        if (ws_num == 1) problem = HW5::getWorkspace1();
        // else if (ws_num == 2) problem = HW5::getWorkspace2(); // Uncomment if HW5::getWorkspace2() exists
        else { std::cerr << "Invalid HW5 workspace number\n"; return; }
    } else {
        std::cerr << "Unsupported workspace: " << ws_name << "\n";
        return;
    }

    if (ws_name == "HW5" && ws_num == 1) {
        problem.x_min = -1.0; problem.x_max = 11.0;
        problem.y_min = -3.0; problem.y_max = 3.0;
    }

    std::vector<std::pair<int, double>> n_r_pairs = {
        {200, 0.5}, {200, 1}, {200, 1.5}, {200, 2},
        {500, 0.5}, {500, 1}, {500, 1.5}, {500, 2}
    };
    std::ofstream results("prm_benchmark.csv");
    results << "n,r,run,valid,path_length,time_ms\n";
    for (const auto& [n, r] : n_r_pairs) {
        for (int run = 0; run < 100; ++run) {
            MyPRM prm;
            prm.setParams(n, r);
            auto start = std::chrono::high_resolution_clock::now();
            Path2D path = prm.plan(problem);
            auto end = std::chrono::high_resolution_clock::now();
            double time_ms = std::chrono::duration<double, std::milli>(end - start).count();
            bool valid = path.valid;
            double length = valid ? path.length() : 0.0;
            results << n << "," << r << "," << run << "," << valid << "," << length << "," << time_ms << "\n";
        }
    }
    results.close();
    std::cout << "Benchmarking complete. Generating boxplot..." << std::endl;
    int ret = system("PYTHONPATH=$(pwd)/scripts python3 ws/hw7/prm_boxplot.py");
    if (ret != 0) {
        std::cerr << "Boxplot generation failed. Please check Python and required packages." << std::endl;
    }
}

// Optional: keep main for standalone use
int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <workspace> <number>\n";
        std::cout << "Example: " << argv[0] << " HW5 1\n";
        return 1;
    }
    run_prm_benchmark(argv[1], std::stoi(argv[2]));
    return 0;
}
