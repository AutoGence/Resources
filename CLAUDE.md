# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the AutoGence Resources documentation site, built with Docusaurus 3.8.1. It provides resources, documentation, and tutorials for users of AutoGence robots (Harfy humanoid robots) and robot parts/components. The site is published at https://resources.autogence.ai.

## Development Commands

All commands are run from the repository root:

### Essential Commands

- **Start dev server**: `npm start` (runs on http://localhost:3001)
- **Build for production**: `npm run build` (outputs to `build/`)
- **Preview production build**: `npm run serve` (serves on http://localhost:3001)
- **Clear cache**: `npm run clear` (useful when troubleshooting build issues)

### Testing Before Committing

Always run a production build before committing documentation changes to catch broken links and markdown errors:

```bash
npm run build
```

Docusaurus config has `onBrokenLinks: 'throw'`, so builds will fail if there are broken internal links.

## Project Architecture

### Directory Structure

```
Resources/
├── .github/workflows/
│   └── deploy.yml             # Automated deployment workflow
├── docs/                      # Documentation markdown files
│   ├── harfy/                # Harfy robot docs
│   ├── components/           # Hardware component APIs
│   ├── domain-controller/
│   ├── quick-start/          # Quick start guides
│   └── api/                  # API references
├── blog/                      # Blog posts
├── src/
│   ├── components/           # React components
│   ├── css/                  # Custom styles (custom.css)
│   └── pages/                # Custom pages
├── static/                    # Static assets (images, files)
├── docusaurus.config.js       # Main site configuration
├── sidebars.js                # Sidebar navigation structure
├── package.json
├── CLAUDE.md
├── CONTRIBUTING.md
├── DEPLOYMENT.md
└── README.md
```

### Key Configuration Files

**docusaurus.config.js**: Main configuration including:
- Site metadata (title, tagline, URL)
- Production URL: https://resources.autogence.ai
- Navigation structure (navbar, footer)
- Local search plugin (@easyops-cn/docusaurus-search-local)
- Edit links point to GitHub repo
- Future flag `v4: true` enabled for Docusaurus v4 compatibility

**sidebars.js**: Defines sidebar navigation structure with two main sidebars:
- `docsSidebar`: Main documentation (Harfy, Components, Domain Controller, API Reference, Tutorials)
- `quickStartSidebar`: Quick start guides

### Adding New Documentation

1. Create markdown/MDX file in appropriate `docs/` subdirectory
2. Add entry to `sidebars.js` in the relevant category
3. Test locally with `npm start`
4. Build to verify: `npm run build`

### Custom Styling

Custom CSS lives in `src/css/custom.css`. The site uses AutoGence branding with custom colors and styles.

## Deployment

### Automated Deployment

The site auto-deploys to https://resources.autogence.ai via GitHub Actions when changes are pushed to `main` branch.

**Workflow**: `.github/workflows/deploy.yml`
- Triggers on push to `main` or manual dispatch
- Node.js 20.18 (required for dependencies)
- Runs `npm ci` then `npm run build` from repository root
- Deploys via SSH to production server
- Sets permissions and reloads nginx

### Required GitHub Secrets

For deployment to work, these secrets must be configured:
- `SSH_PRIVATE_KEY`: Private SSH key for deployment
- `REMOTE_HOST`: Production server hostname
- `REMOTE_USER`: SSH username
- `REMOTE_TARGET`: Target directory (e.g., `/var/www/resources.autogence.ai`)

### Build Output

The build process outputs static files to the `build/` directory at the repository root. This directory is ignored in `.gitignore` and should never be committed.

## Technical Details

### Dependencies

- **Docusaurus**: 3.8.1 (core framework)
- **React**: 19.0.0
- **Search**: @easyops-cn/docusaurus-search-local (local search, no external service)
- **Node.js**: >=18.0 required (20.18 used in CI/CD)

### Build Process

The build process:
1. Processes all markdown/MDX files from `docs/`
2. Applies React components and custom styling
3. Generates static HTML files with client-side hydration
4. Outputs to `build/` directory
5. Includes local search index

### Port Configuration

Development and serve commands use port 3001 (not the default 3000) to avoid conflicts. This is configured in package.json scripts:
- `npm start`: `--port 3001`
- `npm run serve`: `--port 3001`

## Repository Context

- **Main branch**: `main` (use this for PRs)
- **GitHub repo**: https://github.com/AutoGence/Resources
- **Deployment**: Pushes to `main` trigger GitHub Actions workflow that deploys to resources.autogence.ai server via SSH

## Common Issues

### Deployment Permission Errors

If GitHub Actions deployment fails with rsync "Permission denied" errors, the issue is that the target directory on the server is owned by `www-data` but the deployment user needs write access.

**Solution**: The workflow now includes a `SCRIPT_BEFORE` step that changes ownership to the deploy user before rsync runs, then `SCRIPT_AFTER` changes it back to `www-data`.

**Required sudoers permissions** on the server (in `/etc/sudoers.d/github-deploy`):
```
deploy-user ALL=(ALL) NOPASSWD: /usr/bin/chown -R deploy-user\:deploy-user /var/www/resources.autogence.ai*
deploy-user ALL=(ALL) NOPASSWD: /usr/bin/chown -R www-data\:www-data /var/www/resources.autogence.ai*
deploy-user ALL=(ALL) NOPASSWD: /usr/bin/find /var/www/resources.autogence.ai* -type d -exec chmod 755 {} \;
deploy-user ALL=(ALL) NOPASSWD: /usr/bin/find /var/www/resources.autogence.ai* -type f -exec chmod 644 {} \;
deploy-user ALL=(ALL) NOPASSWD: /usr/sbin/nginx -t
deploy-user ALL=(ALL) NOPASSWD: /usr/bin/systemctl reload nginx
```

Replace `deploy-user` with the actual SSH username configured in `REMOTE_USER` secret.
