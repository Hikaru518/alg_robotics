#!/bin/bash
# This script is to generate result pdf files for benchmark data.

./ompl_benchmark_statistics.py ./data/ompl.log -d ./data/ompl.db
./ompl_benchmark_statistics.py -d ./data/ompl.db -p ./results_pdf/results.pdf
