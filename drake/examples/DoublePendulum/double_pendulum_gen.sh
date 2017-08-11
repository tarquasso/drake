#!/bin/bash

# Generates the source files for the DoublePendulum input, state, and output.
me=$(python -c 'import os; print(os.path.realpath("'"$0"'"))')
mydir=$(dirname "$me")
examples=$(dirname "$mydir")
drake=$(dirname "$examples")

namespace="drake::examples::double_pendulum"

source $drake/tools/lcm_vector_gen.sh

gen_vector "double pendulum state vector" theta1 theta2 theta1dot theta2dot
gen_vector "double pendulum input vector" tau1 tau2
gen_vector "double pendulum output vector" theta1 theta2