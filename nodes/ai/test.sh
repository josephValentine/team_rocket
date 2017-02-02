#!/bin/bash

dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

f_array=( "Geometry/Models.py" "Geometry/Functions.py" "Functions.py"
          "Skills.py" "AI.py" )

line=$(printf '%0.1s' "-"{1..80})

for f in "${f_array[@]}"; do
    printf "%s%*.*s\n" "${f}" 0 $((80 - ${#f})) "${line}"
    python "${dir}"/"${f}"
    echo
done
