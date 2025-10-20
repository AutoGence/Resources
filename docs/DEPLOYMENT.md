# Deployment Guide

This guide explains how to set up automated deployment for the AutoGence Resources site using GitHub Actions.

## Overview

The site uses GitHub Actions to automatically build and deploy to `resources.autogence.ai` whenever changes are pushed to the `main` branch.

**Deployment Flow:**
1. Push changes to `main` branch on GitHub
2. GitHub Actions workflow triggers automatically
3. Site is built using `npm run build`
4. Built files are transferred to production server via SSH
5. Permissions are set and nginx is reloaded
6. Site is live at resources.autogence.ai

## Prerequisites

- GitHub repository: `autogence/autogence-resources`
- Production server with nginx configured
- SSH access to production server
- Domain pointing to server: `resources.autogence.ai`

## Initial Setup

### 1. Create GitHub Repository

```bash
# On your local machine, in the resources/docs directory
cd /path/to/resources/docs

# Initialize git (if not already done)
git init

# Add all files
git add .

# Create initial commit
git commit -m "Initial commit: AutoGence Resources documentation site"

# Add GitHub remote (replace with your repo URL)
git remote add origin https://github.com/autogence/autogence-resources.git

# Push to GitHub
git branch -M main
git push -u origin main
```

### 2. Generate SSH Key for Deployment

On your local machine, generate an SSH key pair for GitHub Actions:

```bash
ssh-keygen -t ed25519 -C "github-actions-deploy" -f ~/.ssh/github_deploy_key
```

This creates two files:
- `~/.ssh/github_deploy_key` (private key - for GitHub Secrets)
- `~/.ssh/github_deploy_key.pub` (public key - for server)

### 3. Add Public Key to Production Server

Copy the public key to your production server:

```bash
# Copy public key content
cat ~/.ssh/github_deploy_key.pub

# SSH to your server
ssh your-user@your-server.com

# Add the public key to authorized_keys
mkdir -p ~/.ssh
echo "YOUR_PUBLIC_KEY_CONTENT" >> ~/.ssh/authorized_keys
chmod 600 ~/.ssh/authorized_keys
chmod 700 ~/.ssh

# Test the connection from your local machine
ssh -i ~/.ssh/github_deploy_key your-user@your-server.com
```

### 4. Configure GitHub Secrets

Go to your GitHub repository settings and add these secrets:

**Settings → Secrets and variables → Actions → New repository secret**

Add the following secrets:

| Secret Name | Value | Description |
|-------------|-------|-------------|
| `SSH_PRIVATE_KEY` | Content of `github_deploy_key` | Private SSH key for deployment |
| `REMOTE_HOST` | `your-server-ip-or-domain` | Server hostname or IP address |
| `REMOTE_USER` | `your-username` | SSH username on the server |
| `REMOTE_TARGET` | `/var/www/resources.autogence.ai` | Target directory on server |

**To get the private key content:**
```bash
cat ~/.ssh/github_deploy_key
```

Copy the entire output including `-----BEGIN OPENSSH PRIVATE KEY-----` and `-----END OPENSSH PRIVATE KEY-----`.

### 5. Server Configuration

Ensure your production server has the correct directory structure:

```bash
# On your production server
sudo mkdir -p /var/www/resources.autogence.ai
sudo chown -R your-user:www-data /var/www/resources.autogence.ai
sudo chmod -R 755 /var/www/resources.autogence.ai
```

### 6. Nginx Configuration

Ensure nginx is configured to serve the site (this should already be done via `nginx-resources.conf`):

```nginx
server {
    listen 80;
    listen [::]:80;
    server_name resources.autogence.ai;

    root /var/www/resources.autogence.ai;
    index index.html;

    location / {
        try_files $uri $uri/ /index.html;
    }

    # Enable gzip compression
    gzip on;
    gzip_types text/plain text/css application/json application/javascript text/xml application/xml application/xml+rss text/javascript;
}
```

Reload nginx:
```bash
sudo nginx -t
sudo systemctl reload nginx
```

### 7. Sudo Permissions for Deployment

The deployment script needs to run certain commands with sudo. Add these to your sudoers file:

```bash
sudo visudo -f /etc/sudoers.d/github-deploy
```

Add these lines (replace `your-user` with your actual username):

```
your-user ALL=(ALL) NOPASSWD: /usr/bin/chown -R www-data\:www-data /var/www/resources.autogence.ai*
your-user ALL=(ALL) NOPASSWD: /usr/bin/find /var/www/resources.autogence.ai* -type d -exec chmod 755 {} \;
your-user ALL=(ALL) NOPASSWD: /usr/bin/find /var/www/resources.autogence.ai* -type f -exec chmod 644 {} \;
your-user ALL=(ALL) NOPASSWD: /usr/sbin/nginx -t
your-user ALL=(ALL) NOPASSWD: /usr/bin/systemctl reload nginx
```

Save and exit. Test it:
```bash
sudo -l
```

## Usage

### Automatic Deployment

Simply push to the `main` branch:

```bash
git add .
git commit -m "Update documentation"
git push origin main
```

GitHub Actions will automatically:
1. Build the site
2. Deploy to production
3. Set correct permissions
4. Reload nginx

### Manual Deployment

You can also trigger deployment manually from GitHub:

1. Go to your repository on GitHub
2. Click "Actions" tab
3. Select "Deploy to Production" workflow
4. Click "Run workflow" button
5. Select the `main` branch
6. Click "Run workflow"

### Monitor Deployment

View deployment status:
1. Go to "Actions" tab in your GitHub repository
2. Click on the latest workflow run
3. Watch the real-time logs

## Workflow File

The deployment workflow is defined in `.github/workflows/deploy.yml`:

```yaml
name: Deploy to Production

on:
  push:
    branches:
      - main
  workflow_dispatch:

jobs:
  build-and-deploy:
    runs-on: ubuntu-latest
    steps:
      - Checkout code
      - Setup Node.js
      - Install dependencies
      - Build Docusaurus site
      - Deploy to server via SSH
      - Set permissions and reload nginx
```

## Troubleshooting

### Deployment Fails with Permission Error

**Problem**: Cannot write to `/var/www/resources.autogence.ai`

**Solution**:
```bash
# On server
sudo chown -R your-user:www-data /var/www/resources.autogence.ai
sudo chmod -R 755 /var/www/resources.autogence.ai
```

### SSH Authentication Fails

**Problem**: GitHub Actions cannot connect via SSH

**Solution**:
1. Verify `SSH_PRIVATE_KEY` secret is correctly set
2. Ensure public key is in server's `~/.ssh/authorized_keys`
3. Check SSH key format (should be OpenSSH format)
4. Test SSH connection manually

### Build Fails

**Problem**: `npm run build` fails in GitHub Actions

**Solution**:
1. Test build locally: `npm run build`
2. Check for broken links or markdown errors
3. Review GitHub Actions logs for specific error
4. Ensure all dependencies are in `package.json`

### Nginx Reload Fails

**Problem**: `sudo nginx -t` or `sudo systemctl reload nginx` fails

**Solution**:
1. Check nginx configuration syntax
2. Verify sudoers permissions are correctly set
3. SSH to server and test commands manually

### Site Not Updating

**Problem**: Pushed changes but site shows old content

**Solution**:
1. Check GitHub Actions completed successfully
2. SSH to server and verify files are updated
3. Clear browser cache (Ctrl+F5)
4. Check nginx is serving correct directory

## Rollback

To rollback to a previous version:

1. Find the commit hash of the working version
2. Revert or create a new commit with old content
3. Push to `main` branch to trigger automatic deployment

Or manually on the server:

```bash
# SSH to server
ssh your-user@your-server.com

# Restore from backup (if you have one)
sudo cp -r /backup/resources.autogence.ai/* /var/www/resources.autogence.ai/
sudo systemctl reload nginx
```

## Security Considerations

- **SSH Private Key**: Never commit to repository. Store only in GitHub Secrets.
- **Minimal Permissions**: The deploy user should have minimal permissions on the server.
- **Sudoers File**: Only grant specific commands needed for deployment.
- **Secrets Rotation**: Periodically regenerate SSH keys and update secrets.
- **Branch Protection**: Enable branch protection rules for `main` branch.

## Alternative Deployment Methods

### Manual Deployment (Emergency)

If GitHub Actions is down, deploy manually:

```bash
# On your local machine
cd resources/docs
npm run build

# Deploy via rsync
rsync -avz --delete build/ your-user@your-server:/var/www/resources.autogence.ai/

# SSH to server and fix permissions
ssh your-user@your-server
sudo chown -R www-data:www-data /var/www/resources.autogence.ai
sudo systemctl reload nginx
```

### GitHub Pages (Alternative)

If you want to use GitHub Pages instead:

1. Update `docusaurus.config.js`:
   ```js
   url: 'https://autogence.github.io',
   baseUrl: '/autogence-resources/',
   organizationName: 'autogence',
   projectName: 'autogence-resources',
   ```

2. Deploy to GitHub Pages:
   ```bash
   npm run deploy
   ```

3. Configure custom domain in GitHub repository settings.

## Support

For deployment issues:
- Check GitHub Actions logs first
- Review server logs: `sudo tail -f /var/log/nginx/error.log`
- Contact DevOps team
- Create issue in GitHub repository

---

Last updated: 2025-10-16
