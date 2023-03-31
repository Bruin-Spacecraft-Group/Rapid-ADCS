#!/bin/bash

set -e -o pipefail

function echodo () {
        set -e -o pipefail
        echo "[BEGIN] $@" >&2
        "$@"
        echo "[END  ] $@" >&2
}


export CC=clang-15
export CXX=clang++-15
echodo mkdir -p builds/debug
echodo mkdir -p builds/release

echodo pushd builds/debug
echodo cmake -DCMAKE_BUILD_TYPE=Debug ../..
echodo popd

echodo pushd builds/release
echodo cmake -DCMAKE_BUILD_TYPE=Release ../..
echodo popd
