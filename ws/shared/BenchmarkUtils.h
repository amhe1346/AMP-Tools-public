#pragma once
#include "AMPCore.h"
#include "hw/HW8.h"
#include "../hw8/MyMultiAgentPlanners.h"
#include <vector>
#include <string>

namespace amp {
void runBenchmarks(int min_agents, int max_agents, int num_runs, int n, double r, double pgoal, double epsilon, const std::string& output_prefix);
}
