#!/bin/bash
set -e

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$workspace_dir/install/setup.bash"
params_file="$workspace_dir/src/rb2301_ca2/params/ca2_runtime.yaml"

run_name=sim
calibrate=false
cli_args=("$@")
for ((i=0; i<${#cli_args[@]}; i++)); do
    case "${cli_args[i]}" in
        --run)
            if ((i+1 >= ${#cli_args[@]})); then
                echo "--run requires test1, test2, or full" >&2
                exit 2
            fi
            run_name="${cli_args[i+1]}"
            ;;
        --run=*) run_name="${cli_args[i]#--run=}" ;;
        --calibrate) calibrate=true ;;
        --help|-h)
            exec ros2 run rb2301_ca2 path_planning --help
            ;;
    esac
done
if [[ "$run_name" == sim && "$calibrate" == true ]]; then
    run_name=test1
fi
case "$run_name" in
    sim|test1|test2|full) ;;
    *) echo "--run requires test1, test2, or full" >&2; exit 2 ;;
esac

mkdir -p "$workspace_dir/ca2_runs"
run_dir="$(mktemp -d "$workspace_dir/ca2_runs/${run_name}_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
echo "Saving runtime parameters to $run_dir/params.yaml"
exec ros2 run rb2301_ca2 path_planning \
    --save-params "$run_dir/params.yaml" \
    --ros-args --params-file "$params_file" -- "$@"
