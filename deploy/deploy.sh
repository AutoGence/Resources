#!/usr/bin/env bash
#
# Manual deploy of the AutoGence Resources site to a server.
# Use this for the first deploy to a new host, or when GitHub Actions is down.
# Normal deploys happen automatically on push to main.
#
#   ./deploy.sh user@host [remote-target]
#
# Default remote-target: /var/www/resources.autogence.ai
#
# Builds from the current working tree, so commit first if you want the
# deployed output to match what CI would produce.

set -euo pipefail

REMOTE="${1:-}"
REMOTE_TARGET="${2:-/var/www/resources.autogence.ai}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -z "${REMOTE}" ]]; then
    echo "Usage: $0 user@host [remote-target]" >&2
    echo "  e.g. $0 deploy@203.0.113.10 /var/www/resources.autogence.ai" >&2
    exit 1
fi

cd "${REPO_ROOT}"

if [[ -n "$(git status --porcelain 2>/dev/null)" ]]; then
    echo "WARNING: working tree has uncommitted changes; they will be deployed."
    read -rp "Continue? [y/N] " reply
    [[ "${reply}" =~ ^[Yy]$ ]] || exit 1
fi

echo "==> Installing dependencies (npm ci)"
npm ci

echo "==> Building"
npm run build

if [[ ! -f build/index.html ]]; then
    echo "ERROR: build/index.html missing -- build did not produce a site." >&2
    exit 1
fi

echo "==> Taking ownership of the remote web root"
ssh "${REMOTE}" "sudo chown -R \$USER:\$USER ${REMOTE_TARGET}"

# --delete removes files from previous builds. Docusaurus emits content-hashed
# asset names, so without it the web root grows without bound.
echo "==> Syncing build/ to ${REMOTE}:${REMOTE_TARGET}"
rsync -avz --delete build/ "${REMOTE}:${REMOTE_TARGET}/"

echo "==> Restoring permissions and reloading nginx"
ssh "${REMOTE}" "
    set -e
    sudo chown -R www-data:www-data ${REMOTE_TARGET}
    sudo find ${REMOTE_TARGET} -type d -exec chmod 755 {} \;
    sudo find ${REMOTE_TARGET} -type f -exec chmod 644 {} \;
    sudo nginx -t && sudo systemctl reload nginx
"

echo
echo "==> Deployed."
