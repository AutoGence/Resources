# GitHub Setup Checklist

This checklist guides you through moving the AutoGence Resources site to GitHub and setting up automated deployment.

## Pre-Move Checklist

- [x] Audit codebase for sensitive information
- [x] Clean configuration files
- [x] Update .gitignore
- [x] Create comprehensive README.md
- [x] Set up GitHub Actions workflow
- [x] Create DEPLOYMENT.md guide
- [x] Create CONTRIBUTING.md guide
- [x] Add LICENSE file

## Step 1: Create GitHub Repository

1. Go to https://github.com/autogence (or your organization)
2. Click "New repository"
3. Repository name: `autogence-resources`
4. Description: "Official documentation for AutoGence robotics products"
5. Choose "Public" visibility
6. **Do NOT** initialize with README (we have our own)
7. Click "Create repository"

## Step 2: Push Code to GitHub

From the `resources/docs` directory:

```bash
# Navigate to the resources/docs directory
cd /Users/weliam/Dev/robotics/autogence/mainsite/resources/docs

# Initialize git repository
git init

# Add all files
git add .

# Create initial commit
git commit -m "Initial commit: AutoGence Resources documentation site

- Docusaurus 3.8.1 setup with AutoGence branding
- Documentation for Harfy robot, components, and APIs
- GitHub Actions deployment workflow
- Contributing guidelines and license"

# Add GitHub remote (update with your actual repo URL)
git remote add origin https://github.com/autogence/autogence-resources.git

# Set main as default branch
git branch -M main

# Push to GitHub
git push -u origin main
```

## Step 3: Generate SSH Deployment Key

```bash
# Generate SSH key pair
ssh-keygen -t ed25519 -C "github-actions-deploy" -f ~/.ssh/github_deploy_key

# View private key (for GitHub Secrets)
cat ~/.ssh/github_deploy_key

# View public key (for production server)
cat ~/.ssh/github_deploy_key.pub
```

## Step 4: Add Public Key to Production Server

```bash
# SSH to your production server
ssh your-user@your-server.com

# Add public key to authorized_keys
mkdir -p ~/.ssh
echo "PASTE_PUBLIC_KEY_HERE" >> ~/.ssh/authorized_keys
chmod 600 ~/.ssh/authorized_keys
chmod 700 ~/.ssh

# Exit server
exit

# Test SSH connection with new key
ssh -i ~/.ssh/github_deploy_key your-user@your-server.com
```

## Step 5: Configure GitHub Secrets

Go to your GitHub repository:
**Settings → Secrets and variables → Actions → New repository secret**

Add these four secrets:

### SSH_PRIVATE_KEY
```bash
# Get the private key content
cat ~/.ssh/github_deploy_key
```
Copy the **entire output** including:
- `-----BEGIN OPENSSH PRIVATE KEY-----`
- All the key content
- `-----END OPENSSH PRIVATE KEY-----`

Paste into GitHub secret.

### REMOTE_HOST
Your server IP address or domain name:
```
Example: 123.45.67.89
or: autogence.ai
```

### REMOTE_USER
Your SSH username on the server:
```
Example: deploy
or: your-username
```

### REMOTE_TARGET
Target directory on the server:
```
/var/www/resources.autogence.ai
```

## Step 6: Configure Server Permissions

On your production server:

```bash
# Create target directory
sudo mkdir -p /var/www/resources.autogence.ai

# Set ownership
sudo chown -R your-user:www-data /var/www/resources.autogence.ai

# Set permissions
sudo chmod -R 755 /var/www/resources.autogence.ai
```

## Step 7: Configure Sudo Permissions

On your production server:

```bash
# Create sudoers file for deployment
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

Save and test:
```bash
sudo -l
```

## Step 8: Test Deployment

### Test 1: Trigger Manual Deployment

1. Go to GitHub repository
2. Click "Actions" tab
3. Click "Deploy to Production" workflow
4. Click "Run workflow"
5. Select `main` branch
6. Click "Run workflow" button
7. Watch the deployment progress

### Test 2: Push a Change

```bash
# Make a small change
echo "Test update" >> README.md

# Commit and push
git add README.md
git commit -m "Test: Trigger automatic deployment"
git push origin main

# Watch GitHub Actions
# Go to: https://github.com/autogence/autogence-resources/actions
```

### Test 3: Verify Site

1. Visit https://resources.autogence.ai
2. Verify changes are live
3. Check all pages load correctly
4. Test navigation and search

## Step 9: Repository Configuration

### Enable Branch Protection

1. Go to **Settings → Branches**
2. Click "Add rule"
3. Branch name pattern: `main`
4. Enable:
   - [ ] Require pull request reviews before merging
   - [ ] Require status checks to pass before merging
   - [ ] Require branches to be up to date before merging
5. Save changes

### Configure Repository Settings

1. **Settings → General**
   - [ ] Enable issues
   - [ ] Enable discussions (optional)
   - [ ] Disable wikis (we use Docusaurus)
   - [ ] Disable projects (unless needed)

2. **Settings → Pages**
   - Set to "Deploy from a branch: None" (we use custom deployment)

3. **Settings → Options → Features**
   - [ ] Enable "Automatically delete head branches"

## Step 10: Update Documentation URLs

Update any references to the old repository location:

- [x] README.md - Already updated
- [x] docusaurus.config.js - Already updated
- [x] DEPLOYMENT.md - Already updated
- [x] CONTRIBUTING.md - Already updated

## Step 11: Announce the Move

1. Create a GitHub Discussion or Issue announcing the open-source release
2. Share on AutoGence Discord
3. Update main website (autogence.ai) to link to GitHub repo
4. Consider a blog post about open-sourcing the documentation

## Post-Setup Verification

- [ ] GitHub Actions workflow runs successfully
- [ ] Site deploys automatically on push to main
- [ ] All pages render correctly on resources.autogence.ai
- [ ] Search functionality works
- [ ] Edit links point to correct GitHub URLs
- [ ] No sensitive information exposed
- [ ] README displays properly on GitHub
- [ ] LICENSE file is visible
- [ ] Contributing guidelines are clear

## Troubleshooting

### Deployment fails with SSH error
- Verify SSH_PRIVATE_KEY secret is correctly formatted
- Ensure public key is in server's authorized_keys
- Test SSH connection manually

### Build fails in GitHub Actions
- Check GitHub Actions logs
- Test build locally: `npm run build`
- Verify all dependencies are in package.json

### Site not updating after deployment
- Check GitHub Actions completed successfully
- SSH to server and verify files updated
- Clear browser cache
- Check nginx configuration

### Permission errors on server
```bash
sudo chown -R your-user:www-data /var/www/resources.autogence.ai
sudo chmod -R 755 /var/www/resources.autogence.ai
```

## Maintenance Tasks

### Regular Updates
- Keep dependencies updated: `npm update`
- Review and merge community PRs
- Monitor GitHub Actions for failures
- Rotate SSH keys every 6-12 months

### Security
- Review GitHub Security alerts
- Update Node.js version as needed
- Monitor access logs on server
- Review sudoers permissions periodically

## Support

If you encounter issues:
1. Check DEPLOYMENT.md for detailed troubleshooting
2. Review GitHub Actions logs
3. Check server logs: `sudo tail -f /var/log/nginx/error.log`
4. Open an issue on GitHub
5. Ask on AutoGence Discord

---

**Ready to go?** Follow the steps above in order, and you'll have automated deployments set up in about 30 minutes!

Last updated: 2025-10-16
