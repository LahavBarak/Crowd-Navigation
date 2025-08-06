#!/usr/bin/env bash
set -euo pipefail

# 1-hour runs with these MAX_RUNTIME values:
RUNTIMES=(1.5 1.8 2 2.3 2.5)

# Backup original config
cp config.py config.py.orig

# Header for our results table
echo
echo "MAX_RUNTIME | goals_generated | goals_reached | collisions"
echo "-----------:|:---------------:|:-------------:|:----------:"

for rt in "${RUNTIMES[@]}"; do
  echo "→ Running with MAX_RUNTIME = $rt ..." >&2

  # Rewrite config.py from the original with the new MAX_RUNTIME
  sed -E "s/^MAX_RUNTIME = [0-9.]+/MAX_RUNTIME = $rt/" config.py.orig > config.py

  # Run the simulation and capture its output
  output=$(python3 main.py)

  # Extract the three numbers from the summary line
  if [[ $output =~ time\ up!\ ([0-9]+)\ goals\ generated,\ ([0-9]+)\ goals\ reached,\ ([0-9]+)\ collisions\ occured ]]; then
    goals=${BASH_REMATCH[1]}
    reached=${BASH_REMATCH[2]}
    collisions=${BASH_REMATCH[3]}
  else
    goals="ERR"; reached="ERR"; collisions="ERR"
  fi

  # Print one row of the table
  printf "%-11s | %-15s | %-13s | %-10s\n" \
    "$rt" "$goals" "$reached" "$collisions"
done

# Restore original
mv config.py.orig config.py
echo
echo "✅ Done – restored original config.py" >&2
