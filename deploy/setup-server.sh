#!/usr/bin/env bash
#
# One-time provisioning for a new AutoGence Resources web server.
# Run on the NEW server as a user with sudo. Idempotent -- safe to re-run.
#
#   sudo ./setup-server.sh [deploy-user] [domain]
#
# Defaults: deploy-user = the invoking user, domain = resources.autogence.ai

set -euo pipefail

DEPLOY_USER="${1:-${SUDO_USER:-$(whoami)}}"
DOMAIN="${2:-resources.autogence.ai}"
WEB_ROOT="/var/www/${DOMAIN}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $EUID -ne 0 ]]; then
    echo "ERROR: run with sudo." >&2
    exit 1
fi

echo "==> Provisioning ${DOMAIN}"
echo "    deploy user: ${DEPLOY_USER}"
echo "    web root:    ${WEB_ROOT}"
echo

echo "==> Installing nginx, rsync, certbot"
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y nginx rsync certbot python3-certbot-nginx

echo "==> Creating web root"
mkdir -p "${WEB_ROOT}"
chown -R "${DEPLOY_USER}:www-data" "${WEB_ROOT}"
chmod -R 755 "${WEB_ROOT}"

echo "==> Installing nginx site config"
NGINX_SRC="${SCRIPT_DIR}/nginx-resources.conf"
NGINX_DST="/etc/nginx/sites-available/${DOMAIN}"

if [[ ! -f "${NGINX_SRC}" ]]; then
    echo "ERROR: ${NGINX_SRC} not found." >&2
    exit 1
fi

if [[ -f "${NGINX_DST}" ]]; then
    echo "    ${NGINX_DST} already exists -- backing up, not overwriting a"
    echo "    certbot-managed config blindly."
    cp "${NGINX_DST}" "${NGINX_DST}.bak.$(date +%Y%m%d%H%M%S)"
    echo "    Review ${NGINX_SRC} and merge by hand if needed."
else
    sed "s/resources\.autogence\.ai/${DOMAIN}/g" "${NGINX_SRC}" > "${NGINX_DST}"
    ln -sf "${NGINX_DST}" "/etc/nginx/sites-enabled/${DOMAIN}"
fi

# Ubuntu's stock config serves a placeholder page on the default vhost; it will
# shadow this site if the request arrives without a matching Host header.
if [[ -L /etc/nginx/sites-enabled/default ]]; then
    echo "==> Disabling nginx default site"
    rm /etc/nginx/sites-enabled/default
fi

echo "==> Granting deploy sudo rights (needed by the GitHub Actions workflow)"
SUDOERS_FILE="/etc/sudoers.d/github-deploy"
cat > "${SUDOERS_FILE}" <<EOF
# Managed by deploy/setup-server.sh -- allows the CI deploy job to hand the web
# root back and forth between the deploy user and www-data, and reload nginx.
${DEPLOY_USER} ALL=(ALL) NOPASSWD: /usr/bin/chown -R ${DEPLOY_USER}\\:${DEPLOY_USER} ${WEB_ROOT}*
${DEPLOY_USER} ALL=(ALL) NOPASSWD: /usr/bin/chown -R www-data\\:www-data ${WEB_ROOT}*
${DEPLOY_USER} ALL=(ALL) NOPASSWD: /usr/bin/find ${WEB_ROOT}* -type d -exec chmod 755 {} \\;
${DEPLOY_USER} ALL=(ALL) NOPASSWD: /usr/bin/find ${WEB_ROOT}* -type f -exec chmod 644 {} \\;
${DEPLOY_USER} ALL=(ALL) NOPASSWD: /usr/sbin/nginx -t
${DEPLOY_USER} ALL=(ALL) NOPASSWD: /usr/bin/systemctl reload nginx
EOF
chmod 0440 "${SUDOERS_FILE}"
visudo -c -f "${SUDOERS_FILE}"

echo "==> Testing and reloading nginx"
nginx -t
systemctl enable nginx
systemctl reload nginx

echo
echo "==> Server provisioned."
echo
echo "Next steps:"
echo "  1. Point DNS for ${DOMAIN} at this server's public IP."
echo "  2. Upload the site:  ./deploy.sh ${DEPLOY_USER}@<this-host>"
echo "  3. Issue TLS:        sudo certbot --nginx -d ${DOMAIN}"
echo "  4. Update the GitHub secrets REMOTE_HOST / REMOTE_USER / REMOTE_TARGET"
echo "     and add the CI public key to ~${DEPLOY_USER}/.ssh/authorized_keys."
