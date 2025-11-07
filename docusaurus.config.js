// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'AutoGence Resources',
  tagline: 'Documentation, tutorials, and guides for Harfy robots and embodied AI components',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://resources.autogence.ai',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'AutoGence', // Usually your GitHub org/user name.
  projectName: 'Resources', // Usually your repo name.

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Enable "Edit this page" links that point to GitHub
          editUrl:
            'https://github.com/AutoGence/Resources/tree/main/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Enable "Edit this page" links that point to GitHub
          editUrl:
            'https://github.com/AutoGence/Resources/tree/main/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  plugins: [
    [
      require.resolve('@easyops-cn/docusaurus-search-local'),
      {
        hashed: true,
        language: ['en'],
        highlightSearchTermsOnTargetPage: true,
        explicitSearchResultPath: true,
        docsRouteBasePath: '/docs',
        blogRouteBasePath: '/blog',
      },
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Docs configuration
      docs: {
        sidebar: {
          hideable: true,
          autoCollapseCategories: true,
        },
      },
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      // Hide the table of contents on the right
      tableOfContents: {
        minHeadingLevel: 2,
        maxHeadingLevel: 4,
      },
      navbar: {
        title: 'AutoGence Resources',
        logo: {
          alt: 'AutoGence Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'docsSidebar',
            position: 'left',
            label: 'Documentation',
          },
          {
            type: 'docSidebar',
            sidebarId: 'quickStartSidebar',
            position: 'left',
            label: 'Quick Start',
          },
          {
            href: 'https://autogence.ai',
            label: 'Main Site',
            position: 'right',
          },
          {
            href: 'https://github.com/autogence',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Documentation',
            items: [
              {
                label: 'Harfy Documentation',
                to: '/docs/harfy/getting-started',
              },
              {
                label: 'Domain Controller',
                to: '/docs/domain-controller/setup',
              },
              {
                label: 'Quick Start Guides',
                to: '/docs/quick-start/harfy-10min',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Discord',
                href: 'https://discord.gg/autogence',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/autogence',
              },
            ],
          },
          {
            title: 'AutoGence',
            items: [
              {
                label: 'Main Website',
                href: 'https://autogence.ai',
              },
              {
                label: 'Store',
                href: 'https://store.autogence.ai',
              },
              {
                label: 'About',
                href: 'https://autogence.ai#about',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} AutoGence, Inc. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
