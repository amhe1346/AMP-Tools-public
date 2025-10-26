#pragma once
#include "AMPCore.h"
#include <vector>
#include "../shared/MyTranslationalCSpace.h"

namespace amp {
class MyCentralPlanner {
public:
    MyCentralPlanner();
    ~MyCentralPlanner();
    MultiAgentPath2D plan(const MultiAgentProblem2D& problem,
                        const std::vector<MyTranslationalCSpace>& cached_cspaces,
                        int n, double r, double pgoal, double epsilon);
    MultiAgentPath2D plan(const MultiAgentProblem2D& problem,
                        int n, double r, double pgoal, double epsilon);
    MultiAgentPath2D plan(const MultiAgentProblem2D& problem);
};

class MyDecentralPlanner {
public:
    MyDecentralPlanner();
    ~MyDecentralPlanner();
    MultiAgentPath2D plan(const MultiAgentProblem2D& problem,
                        const std::vector<MyTranslationalCSpace>& cached_cspaces,
                        int n, double r, double pgoal, double epsilon);
    MultiAgentPath2D plan(const MultiAgentProblem2D& problem);
};
}
