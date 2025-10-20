// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * AutoGence Resources Sidebar Configuration
 * Starting with actuator control API documentation
 *
 * @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Main Documentation Sidebar
  docsSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Harfy Robot',
      collapsed: false,
      items: [
        'harfy/getting-started',
      ],
    },
    {
      type: 'category',
      label: 'Components',
      collapsed: false,
      items: [
        'components/controller-api',
        'components/actuator-api',
      ],
    },
    {
      type: 'category',
      label: 'Domain Controller',
      collapsed: false,
      items: [
        'domain-controller/setup',
      ],
    },
    {
      type: 'category',
      label: 'API Reference',
      items: [
        'api/intro',
      ],
    },
    {
      type: 'category',
      label: 'Tutorial Basics',
      items: [
        'tutorial-basics/create-a-document',
        'tutorial-basics/create-a-blog-post',
        'tutorial-basics/markdown-features',
        'tutorial-basics/deploy-your-site',
        'tutorial-basics/congratulations',
      ],
    },
    {
      type: 'category',
      label: 'Tutorial Extras',
      items: [
        'tutorial-extras/manage-docs-versions',
        'tutorial-extras/translate-your-site',
      ],
    },
  ],

  // Quick Start Guides
  quickStartSidebar: [
    'quick-start/intro',
    'quick-start/harfy-setup',
    'quick-start/harfy-10min',
  ],
};

export default sidebars;
