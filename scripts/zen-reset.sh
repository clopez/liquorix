#!/usr/bin/env bash

echo "Resetting git..."
git reset 'HEAD' && git checkout -f && git checkout -b master-old -f && git branch -D master && git checkout -b master origin/master && git branch -D master-old
echo "Done!"

