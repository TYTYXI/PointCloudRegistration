set(sources
        L2_PE_Problem.h
        L2_PE_Problem.cpp
        L2_PE_gpu.cuh
        L2_PE_gpu.cu
        L2_SimpleProblem.h
        L2_SimpleProblem.cpp
)

extend_ons_target(Registration
        SOURCES ${sources}
)