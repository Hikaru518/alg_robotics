#!/bin/bash
# This script is to generate result pdf files for benchmark data.

./ompl_benchmark_statistics.py ./data/C_uniform_sampler.log -d ./data/C_uniform_sampler.db
./ompl_benchmark_statistics.py -d ./data/C_uniform_sampler.db -p ./results_pdf/C_uniform_sampler_results.pdf

./ompl_benchmark_statistics.py ./data/T_uniform_sampler.log -d ./data/T_uniform_sampler.db
./ompl_benchmark_statistics.py -d ./data/T_uniform_sampler.db -p ./results_pdf/T_uniform_sampler_results.pdf

rm *.log
rm *.console
