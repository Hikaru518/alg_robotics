#!/bin/bash
# This script is to generate result pdf files for benchmark data.

./ompl_benchmark_statistics.py ./data/uniform_sampler.log -d ./data/uniform_sampler.db
./ompl_benchmark_statistics.py -d ./data/uniform_sampler.db -p ./results_pdf/uniform_sampler_results.pdf

./ompl_benchmark_statistics.py ./data/gaussian_sampler.log -d ./data/gaussian_sampler.db
./ompl_benchmark_statistics.py -d ./data/gaussian_sampler.db -p ./results_pdf/gaussian_sampler.pdf

./ompl_benchmark_statistics.py ./data/obstaclebased_sampler.log -d ./data/obstaclebased_sampler.db
./ompl_benchmark_statistics.py -d ./data/obstaclebased_sampler.db -p ./results_pdf/obstaclebased_sampler.pdf

./ompl_benchmark_statistics.py ./data/maxclearance_sampler.log -d ./data/maxclearance_sampler.db
./ompl_benchmark_statistics.py -d ./data/maxclearance_sampler.db -p ./results_pdf/maxclearance_sampler.pdf
