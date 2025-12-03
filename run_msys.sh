#!/bin/bash
export ALLEGRO_LINK_DIR=$(pwd)/allegro/lib
export ALLEGRO_INCLUDE_DIR=$(pwd)/allegro/include
cargo run --release --features use_user_settings
