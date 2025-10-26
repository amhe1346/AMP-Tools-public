#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW8.h"


// This is file is loaded from the shared/ directory
// Overwrite with your MySamplingBasedPlanners.h and MySamplingBasedPlanners.cpp from hw7
#include "../shared/MySamplingBasedPlanners.h"
#include "../shared/MyTranslationalCSpace.h"



class MyCentralPlanner : public amp::CentralizedMultiAgentRRT {
public:
    virtual ~MyCentralPlanner();
    virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
    amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem, int n, double r, double pgoal, double epsilon);

    // New: plan with precomputed C-spaces
    amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem,
                               const std::vector<amp::MyTranslationalCSpace>& cspaces,
                               int n, double r, double pgoal, double epsilon);
};


class MyDecentralPlanner : public amp::DecentralizedMultiAgentRRT {
public:
    virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;
    // New: plan with precomputed C-spaces
    amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem,
                               const std::vector<amp::MyTranslationalCSpace>& cspaces,
                               int n, double r, double pgoal, double epsilon);
};