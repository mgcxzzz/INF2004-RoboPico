#!/bin/sh
# Split one console log that covers several test IDs into one file per ID.
#   sh testing/split_log.sh <full-log> <logs-dir> <YYYY-MM-DD> <what>
# Every line that names a test ID (S1-U01, S5-C03, INT-02, SYS-U01, ...)
# goes into <logs-dir>/<date>_<ID>_<what>.txt, after a short header.
# Lines naming no ID are kept only in the full log.
set -e
[ $# -eq 4 ] || { echo "usage: $0 <full-log> <logs-dir> <YYYY-MM-DD> <what>"; exit 1; }
log=$1; dir=$2; date=$3; what=$4
commit=$(git rev-parse --short HEAD 2>/dev/null || echo "?")
ids=$(tr -d '\r' < "$log" | grep -oE '(S[1-5]-[UMC][0-9]{2}|INT-[0-9]{2}|SYS-[A-Z]?[0-9]{2})' | sort -u)
[ -n "$ids" ] || { echo "no test IDs found in $log"; exit 1; }
for id in $ids; do
    out="$dir/${date}_${id}_${what}.txt"
    {
        echo "$id  ($what)"
        echo "Date: $date   Commit: $commit   Source: $(basename "$log")"
        echo "----- lines for $id -----"
        tr -d '\r' < "$log" | grep -E "(^|[^A-Z0-9-])$id([^0-9]|$)"
    } > "$out"
    echo "wrote $out"
done
