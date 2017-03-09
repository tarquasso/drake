#!/bin/bash

# Generates the source files for the SoftPaddle input, state, and output.
me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
examples=$(dirname "$mydir")
drake=$(dirname "$examples")

namespace="drake::examples::soft_paddle"

source $drake/tools/lcm_vector_gen.sh

gen_vector "soft_paddle state vector" theta1 theta2 theta1dot theta2dot
gen_vector "soft_paddle input vector" tau
gen_vector "soft_paddle output vector" theta1 theta2
