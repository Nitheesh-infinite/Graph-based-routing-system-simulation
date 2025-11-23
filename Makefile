CXX = g++
# Added -IPhase-2 so headers in the second folder are found
CXXFLAGS := -std=c++17 -I. -IPhase-1 -IPhase-2 -Wall -Wextra
RELEASE_FLAGS := -O2
DEBUG_FLAGS := -g -O0
# Address Sanitizer (detects memory leaks and out-of-bounds access)
ASAN_FLAGS := -fsanitize=address -g -O1 

# --- SOURCE DEFINITIONS ---


# Phase 1 specific sources (includes its own KDT and KNN)
P1_SOURCES = Phase-1/SampleDriver.cpp \
             Phase-1/graph.cpp \
             Phase-1/kdt.cpp \
             Phase-1/knn.cpp \
             Phase-1/srtp.cpp

# Phase 2 specific sources (plus the shared SRTP)
P2_SOURCES = Phase-2/SampleDriver.cpp \
             Phase-2/graph.cpp \
             Phase-2/K_shortest_paths.cpp \
             Phase-2/K_shortest_path_heustiric.cpp \
             Phase-2/approx_shortest.cpp \
             Phase-2/srtp.cpp
P3_SOURCES = Phase-3/SampleDriver.cpp \
             Phase-3/fleet.cpp \
             Phase-2/graph.cpp \
             Phase-2/srtp.cpp \
             Phase-2/K_shortest_paths.cpp \
             Phase-2/K_shortest_path_heustiric.cpp \
             Phase-2/approx_shortest.cpp

# Target to build Phase 3
phase3: $(P3_SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@


# --- TARGETS ---
.PHONY: all phase1 phase2 debug asan debug2 asan2 clean

# Default target (Release build for Phase 1)
all: phase1

# --- PHASE 1 BUILDS ---

phase1: $(P1_SOURCES)
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) $^ -o $@

# Debug build for Phase 1
debug: CXXFLAGS += $(DEBUG_FLAGS)
debug: clean_artifacts phase1

# ASAN build for Phase 1
asan: CXXFLAGS += $(ASAN_FLAGS)
asan: clean_artifacts phase1

# --- PHASE 2 BUILDS ---

phase2: $(P2_SOURCES)
	$(CXX) $(CXXFLAGS) $(RELEASE_FLAGS) $^ -o $@

# Debug build for Phase 2
debug2: CXXFLAGS += $(DEBUG_FLAGS)
debug2: clean_artifacts phase2

# ASAN build for Phase 2
asan2: CXXFLAGS += $(ASAN_FLAGS)
asan2: clean_artifacts phase2

# --- CLEANUP ---

# Helper to avoid full re-make logic on clean
clean_artifacts:
	rm -f phase1 phase2 output.json

clean: clean_artifacts