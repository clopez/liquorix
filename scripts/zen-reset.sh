#!/usr/bin/env bash

echo "Resetting the repository..."
git add .; git reset --hard; git branch -M master master-old; git checkout -b master origin/master; git branch -D master-old

echo "Done!"

