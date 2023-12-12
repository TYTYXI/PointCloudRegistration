set(sources
        AlgorithmObject.h
        AlgorithmObject.cpp
        TeachingLearningBasedOptimization.h
        TeachingLearningBasedOptimization.cpp
        MultipleClassTeachingLearningBasedOptimization.h
        MultipleClassTeachingLearningBasedOptimization.cpp
        CuckooSearch.h
        CuckooSearch.cpp
        WhaleOptimization.h
        WhaleOptimization.cpp
        ArtificialBeeColony.h
        ArtificialBeeColony.cpp
        TeachingLearningBasedWhaleOptimiziotn.h
        TeachingLearningBasedWhaleOptimiziotn.cpp
        SmallGroupTeachingLearningBasedOptimization.h
        SmallGroupTeachingLearningBasedOptimization.cpp
        MultipleWhaleGroupsTeachingLearningBasedWhaleOptimiziotn.h
        MultipleWhaleGroupsTeachingLearningBasedWhaleOptimiziotn.cpp
#        SingerInitMCTLBO.h
#        SingerInitMCTLBO.cpp
        TeachingLearningBasedOptimizationMatrixType.h
        TeachingLearningBasedOptimizationMatrixType.cpp
)

extend_ons_target(OptimizationAlgorithms
        SOURCES ${sources}
)