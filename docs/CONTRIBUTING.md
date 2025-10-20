# Contributing to AutoGence Resources

Thank you for your interest in contributing to AutoGence Resources! This documentation helps developers and users work with Harfy robots and AutoGence products. We welcome contributions from the community.

## How to Contribute

### Types of Contributions

We welcome various types of contributions:

- **Documentation improvements**: Fix typos, clarify instructions, add examples
- **New tutorials**: Create step-by-step guides for common use cases
- **Code examples**: Add working code snippets and sample projects
- **Bug reports**: Report broken links, incorrect information, or outdated content
- **Feature requests**: Suggest new documentation topics or improvements
- **Translations**: Help translate documentation to other languages (future)

### Getting Started

1. **Fork the repository** on GitHub
2. **Clone your fork** locally:
   ```bash
   git clone https://github.com/YOUR_USERNAME/autogence-resources.git
   cd autogence-resources
   ```
3. **Install dependencies**:
   ```bash
   npm install
   ```
4. **Create a branch** for your changes:
   ```bash
   git checkout -b fix-typo-in-harfy-setup
   ```
5. **Make your changes** and test locally:
   ```bash
   npm start
   ```
6. **Commit your changes**:
   ```bash
   git add .
   git commit -m "Fix typo in Harfy setup guide"
   ```
7. **Push to your fork**:
   ```bash
   git push origin fix-typo-in-harfy-setup
   ```
8. **Open a Pull Request** on GitHub

## Documentation Guidelines

### Writing Style

- **Clear and concise**: Use simple language, avoid jargon when possible
- **Active voice**: "Click the button" instead of "The button should be clicked"
- **Present tense**: "The robot moves" instead of "The robot will move"
- **Second person**: Address the reader as "you"
- **Consistent terminology**: Use the same terms throughout (e.g., "Harfy" not "harfy robot")

### Structure

- **Start with context**: Explain what the reader will learn
- **Break into sections**: Use headings (##, ###) to organize content
- **Use lists**: Bullet points and numbered lists improve readability
- **Add examples**: Include code snippets, commands, and screenshots
- **End with next steps**: Link to related documentation

### Code Examples

- **Test all code**: Ensure examples work before submitting
- **Use syntax highlighting**: Specify language in code blocks
  ````markdown
  ```python
  import autogence
  robot = autogence.connect("harfy-123")
  ```
  ````
- **Add comments**: Explain non-obvious code
- **Show complete examples**: Include imports and setup code

### Markdown Best Practices

- Use `#` for page title (only one per page)
- Use `##` for major sections, `###` for subsections
- Use backticks for `inline code` and commands
- Use **bold** for emphasis, *italics* sparingly
- Add blank lines between paragraphs and sections
- Use relative links for internal pages: `[Setup Guide](./setup.md)`

## File Organization

```
docs/
├── harfy/                  # Harfy robot documentation
│   ├── getting-started.md
│   └── troubleshooting.md
├── components/             # Hardware components
│   ├── controller-api.md
│   └── actuator-api.md
├── quick-start/           # Quick start guides
│   ├── intro.md
│   └── harfy-10min.md
└── api/                   # API references
    └── intro.md
```

### Creating New Documentation

1. Choose the appropriate directory
2. Create a new `.md` or `.mdx` file
3. Add frontmatter at the top:
   ```markdown
   ---
   sidebar_position: 1
   title: My New Guide
   description: Brief description for SEO
   ---
   ```
4. Update `sidebars.js` if needed
5. Test locally with `npm start`

## Pull Request Process

### Before Submitting

- [ ] Test locally with `npm start`
- [ ] Check for broken links
- [ ] Verify code examples work
- [ ] Fix any spelling/grammar errors
- [ ] Ensure builds successfully with `npm run build`
- [ ] Review your changes in the browser

### PR Description

Include in your pull request:

1. **What**: Brief description of changes
2. **Why**: Reason for the change
3. **How**: Approach taken (if relevant)
4. **Testing**: How you tested the changes
5. **Screenshots**: For visual changes

Example:
```markdown
## What
Fixed broken links in the Harfy setup guide

## Why
Users reported 404 errors when clicking API reference links

## How
Updated links to point to the new API documentation structure

## Testing
- Tested all links manually
- Ran `npm run build` successfully
- Verified in local development server
```

### Review Process

1. Maintainers will review your PR within 3-5 business days
2. Address any requested changes
3. Once approved, maintainers will merge your PR
4. Your changes will be deployed automatically to resources.autogence.ai

## Reporting Issues

Found a problem? Please open an issue on GitHub:

1. Go to [Issues](https://github.com/autogence/autogence-resources/issues)
2. Click "New Issue"
3. Choose appropriate template (Bug Report or Feature Request)
4. Fill out the template completely
5. Add relevant labels

### Bug Report Template

```markdown
**Description**
Clear description of the issue

**Location**
Link to the page with the issue

**Expected behavior**
What should happen

**Actual behavior**
What actually happens

**Steps to reproduce**
1. Go to...
2. Click on...
3. See error

**Screenshots**
If applicable

**Environment**
- Browser: Chrome 120
- OS: macOS 14
- Device: Desktop
```

## Code of Conduct

### Our Standards

- **Be respectful**: Treat everyone with respect and kindness
- **Be constructive**: Provide helpful feedback and suggestions
- **Be inclusive**: Welcome diverse perspectives and backgrounds
- **Be patient**: Remember that everyone was a beginner once
- **Be professional**: Keep discussions focused on the topic

### Unacceptable Behavior

- Harassment or discrimination of any kind
- Trolling, insulting, or derogatory comments
- Personal or political attacks
- Publishing private information
- Other conduct inappropriate in a professional setting

### Enforcement

Violations of the code of conduct may result in:
1. Warning from maintainers
2. Temporary ban from the project
3. Permanent ban from the project

Report issues to: conduct@autogence.ai

## Development Setup

### Prerequisites

- Node.js 18.0 or higher
- npm or yarn
- Git
- Text editor (VS Code recommended)

### Recommended VS Code Extensions

- Markdown All in One
- Markdown Preview Enhanced
- Code Spell Checker
- Prettier - Code formatter

### Local Development Workflow

```bash
# Start development server
npm start

# In another terminal, make changes to files
# Changes will hot-reload automatically

# Build to test production version
npm run build

# Serve production build locally
npm run serve

# Clear cache if needed
npm run clear
```

### Troubleshooting Development Issues

**Port already in use**
```bash
# Kill process on port 3001
npx kill-port 3001
npm start
```

**Build errors**
```bash
# Clear cache and rebuild
npm run clear
rm -rf node_modules package-lock.json
npm install
npm run build
```

**Hot reload not working**
```bash
# Restart development server
# Stop with Ctrl+C, then:
npm start
```

## Style Guide

### Headings

```markdown
# Page Title (H1) - Use once per page

## Major Section (H2)

### Subsection (H3)

#### Minor Heading (H4) - Use sparingly
```

### Links

```markdown
<!-- External links -->
[Visit AutoGence](https://autogence.ai)

<!-- Internal links (relative) -->
[Setup Guide](./setup.md)
[API Reference](../api/intro.md)

<!-- Links with descriptions -->
[Harfy Robot](https://autogence.ai/harfy "20 DOF Humanoid Robot")
```

### Images

```markdown
<!-- Basic image -->
![Alt text](./images/robot.png)

<!-- Image with title -->
![Harfy Robot](./images/harfy.jpg "Harfy humanoid robot")

<!-- Centered image -->
<div style={{textAlign: 'center'}}>
  <img src="./images/robot.png" alt="Robot" />
</div>
```

### Admonitions

```markdown
:::tip
Useful tip for readers
:::

:::note
Additional information
:::

:::warning
Important warning
:::

:::danger
Critical information
:::

:::info
Informational note
:::
```

### Code Blocks

````markdown
```bash
# Bash commands
npm install
```

```python
# Python code
import autogence
robot = autogence.Robot()
```

```javascript
// JavaScript code
const robot = new Robot();
```

```json
{
  "name": "Harfy",
  "version": "1.0.0"
}
```
````

## Getting Help

- **Documentation**: Read the [Docusaurus documentation](https://docusaurus.io)
- **Discord**: Join our [community Discord](https://discord.gg/autogence)
- **Issues**: Check existing [GitHub issues](https://github.com/autogence/autogence-resources/issues)
- **Email**: Contact us at docs@autogence.ai

## Recognition

Contributors will be:
- Listed in the repository's contributors page
- Credited in release notes for significant contributions
- Invited to join the AutoGence community Discord

Thank you for helping make AutoGence Resources better!

---

**Questions?** Open an issue or reach out on Discord!
