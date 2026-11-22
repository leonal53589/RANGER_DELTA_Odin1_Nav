#!/usr/bin/env bash
# Build the opt-in Nav2 Humble controller in an isolated overlay.
set -euo pipefail

project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
workspace="$project_dir/build/nav2_goal_time_test"
patch_file="$project_dir/scripts/nav2_goal_time_test/controller.patch"
version=1.1.20
archive="$workspace/navigation2-$version.tar.gz"
package="$workspace/src/nav2_controller"
marker="$workspace/install/nav2_controller/share/nav2_controller/odin_goal_time_test"
mkdir -p "$workspace/src"

set +u
source /opt/ros/humble/setup.bash
set -u
if [[ ! -f "$archive" ]]; then
    temporary_archive=$(mktemp "$workspace/archive.XXXXXX")
    trap 'rm -f -- "$temporary_archive"' EXIT
    curl --fail --silent --show-error --location \
        "https://codeload.github.com/ros-navigation/navigation2/tar.gz/refs/tags/$version" \
        --output "$temporary_archive"
    mv -- "$temporary_archive" "$archive"
fi
printf '%s  %s\n' c965b7a36ef48cd7f35f01c1f98883741693d195dae582232d6d0444d2eedab6 "$archive" |
    sha256sum --check --status

tar -xzf "$archive" --directory "$workspace/src" --strip-components=1 \
    "navigation2-$version/nav2_controller"
patch --directory="$package" --strip=1 --batch --forward --input="$patch_file"
rm -f -- "$marker"
colcon --log-base "$workspace/log" build --base-paths "$package" \
    --build-base "$workspace/colcon" --install-base "$workspace/install" \
    --packages-select nav2_controller --allow-overriding nav2_controller \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
printf 'Nav2 %s goal TF timing test with per-goal checker selection\n' "$version" > "$marker"
printf 'Test overlay: %s\n' "$workspace/install/local_setup.bash"
