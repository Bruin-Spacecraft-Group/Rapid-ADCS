#!/bin/bash

set -e -o pipefail

function echodo () {
        set -e -o pipefail
        echo "[BEGIN] $@" >&2
        "$@"
        echo "[END  ] $@" >&2
}


export CC=clang-13
export CXX=clang++-13
echodo mkdir -p builds/debug
echodo mkdir -p builds/release

echodo pushd builds/debug
echodo cmake -DCMAKE_BUILD_TYPE=Debug ../..
echodo popd

echodo pushd builds/release
echodo cmake -DCMAKE_BUILD_TYPE=Release ../..
echodo popd
