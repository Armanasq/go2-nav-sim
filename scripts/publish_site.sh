#!/bin/bash
# Build the site, check it, and publish it to the gh-pages branch (GitHub Pages "deploy from branch").
#   scripts/publish_site.sh            needs: python3 with markdown + pyyaml; for the check, a python with playwright
# The CI workflow does the same on push when GitHub Actions is available; this is the manual path.
set -eo pipefail
cd "$(dirname "$0")/.."
PY_CHECK=${PY_CHECK:-python3}
python3 site/build.py
"$PY_CHECK" tools/check_site.py _site ${CHROME:+--chrome "$CHROME"} --write     # refuses to publish a site with broken links or errors
python3 site/build.py                                                            # rebuild so the evidence page shows this check
SHA=$(git rev-parse --short HEAD); TMP=$(mktemp -d)
cp -r _site/. "$TMP"; cd "$TMP"
git init -q -b gh-pages && git add -A && git commit -q -m "Site build from main@$SHA"
git push -f "$(git -C "$OLDPWD" remote get-url origin)" gh-pages
echo "published main@$SHA to gh-pages"
