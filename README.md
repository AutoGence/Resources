# AutoGence Resources
  
> Official documentation, tutorials, and API references for AutoGence robotics products

This repository powers [resources.autogence.ai](https://resources.autogence.ai) - the comprehensive documentation portal for Harfy humanoid robots, embodied AI components, and the AutoGence software platform.

## What's Inside

- **Harfy Robot Documentation**: Getting started guides, setup instructions, and troubleshooting
- **Component APIs**: Documentation for Robot Domain Controller, smart actuators, and sensors
- **Software Platform**: PKI system, OTA updates, and fleet management guides
- **Quick Start Guides**: Fast-track tutorials to get up and running in minutes
- **Tutorials**: Step-by-step learning paths for developers

## Tech Stack

This site is built with [Docusaurus](https://docusaurus.io/), a modern static site generator optimized for documentation.

- **Framework**: Docusaurus 3.8.1
- **React**: 19.0.0
- **Search**: Local search plugin by @easyops-cn
- **Styling**: Custom CSS with AutoGence branding

## Local Development

### Prerequisites

- Node.js 18.0 or higher
- npm or yarn package manager

### Installation

```bash
npm install
```

### Start Development Server

```bash
npm start
```

This command starts a local development server at `http://localhost:3001` with hot reload.

### Build for Production

```bash
npm run build
```

This generates static content into the `build/` directory that can be served by any static hosting service.

### Preview Production Build

```bash
npm run serve
```

Serves the production build locally at `http://localhost:3001` for testing.

### Clear Cache

```bash
npm run clear
```

Clears Docusaurus cache and generated files. Useful for troubleshooting build issues.

## Project Structure

```
Resources/
├── .github/
│   └── workflows/
│       └── deploy.yml        # Automated deployment workflow
├── docs/                     # Documentation content (Markdown/MDX)
│   ├── harfy/               # Harfy robot documentation
│   ├── components/          # Hardware component APIs
│   ├── domain-controller/   # Domain controller setup
│   ├── quick-start/         # Quick start guides
│   └── api/                 # API references
├── blog/                    # Blog posts and tutorials
├── src/
│   ├── components/          # React components
│   ├── css/                 # Custom styles
│   └── pages/               # Custom pages
├── static/                  # Static assets (images, files)
├── docusaurus.config.js     # Site configuration
├── sidebars.js              # Sidebar navigation structure
├── package.json             # Dependencies and scripts
├── CLAUDE.md                # Guidance for Claude Code
├── CONTRIBUTING.md          # Contributing guidelines
├── DEPLOYMENT.md            # Deployment documentation
└── README.md                # This file
```

## Contributing

We welcome contributions to improve our documentation! Here's how you can help:

1. **Fork the Repository**: Click the "Fork" button at the top right
2. **Clone Your Fork**:
   ```bash
   git clone https://github.com/YOUR_USERNAME/Resources.git
   cd Resources
   ```
3. **Create a Branch**:
   ```bash
   git checkout -b improve-docs
   ```
4. **Make Your Changes**: Edit documentation files in the `docs/` directory
5. **Test Locally**:
   ```bash
   npm start
   ```
6. **Build to verify**:
   ```bash
   npm run build
   ```
7. **Commit and Push**:
   ```bash
   git add .
   git commit -m "Improve XYZ documentation"
   git push origin improve-docs
   ```
8. **Open a Pull Request**: Go to the original repository and click "New Pull Request"

### Documentation Guidelines

- Use clear, concise language
- Include code examples where applicable
- Add images/screenshots to illustrate complex concepts (place in `static/img/`)
- Test all commands and code snippets
- Follow the existing document structure and formatting
- Use Markdown or MDX format
- Update `sidebars.js` when adding new pages
- Ensure builds succeed with `npm run build` before submitting PR

## Deployment

This site is automatically deployed to [resources.autogence.ai](https://resources.autogence.ai) via GitHub Actions when changes are merged to the `main` branch.

### Deployment Process

1. Merge pull request to `main` branch
2. GitHub Actions triggers build workflow
3. Site is built using `npm run build`
4. Static files are deployed to production server via SSH
5. Site is live at resources.autogence.ai

See `.github/workflows/deploy.yml` for deployment configuration.

### Required GitHub Secrets

For the deployment workflow to function, the following secrets must be configured in GitHub repository settings:

- `SSH_PRIVATE_KEY`: Private SSH key for deployment server access
- `REMOTE_HOST`: Production server hostname or IP address
- `REMOTE_USER`: SSH username for deployment
- `REMOTE_TARGET`: Target directory on server (e.g., `/var/www/resources.autogence.ai`)

## License

Copyright © 2025 AutoGence L.L.C. All rights reserved.

This documentation is open-source and available for community contributions, but the AutoGence brand, logos, and product names are trademarks of AutoGence L.L.C.

## Community & Support

- **Discord**: [Join our community](https://discord.gg/autogence)
- **GitHub**: [AutoGence repositories](https://github.com/autogence)
- **Main Website**: [autogence.ai](https://autogence.ai)
- **Store**: [store.autogence.ai](https://store.autogence.ai)

## About AutoGence

AutoGence is pioneering accessible humanoid robotics through open-source platforms, embodied AI components, and developer-friendly software. Our flagship product, Harfy, is a 20 DOF humanoid robot designed for education, research, and custom applications.

---

Built with [Docusaurus](https://docusaurus.io/) 🦖
