import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Robotics Book',
  tagline: 'A comprehensive guide to robotics',
  favicon: 'img/favicon.ico',

  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.13.24/dist/katex.min.css',
      type: 'text/css',
      integrity:
        'sha384-odtC+0UGzzFL/6PNoE8rX/SPcQDXBJ+uRepguP4QkPCm2LBxH3FA3y+fKSiJ+AmM',
      crossorigin: 'anonymous',
    },
    {
      href: 'https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700;800&display=swap',
      type: 'text/css',
    },
    {
      href: 'https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@300;400;500;600;700&display=swap',
      type: 'text/css',
    },
    {
      href: 'https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&display=swap',
      type: 'text/css',
    },
  ],

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://mutahirshah11.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'mutahirshah11', // Usually your GitHub org/user name.
  projectName: 'Tbook', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenAnchors: 'throw',
  onBrokenMarkdownLinks: 'throw',

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
      {
        docs: {
          sidebarPath: './sidebars.ts',
          remarkPlugins: [remarkMath],
          rehypePlugins: [rehypeKatex],
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: [
            './src/css/design-system.css',
            './src/css/custom.css',
            './src/css/futuristic-theme.css',
          ],
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: true,
      respectPrefersColorScheme: false,
    },
    navbar: {
      title: 'RoboLearn',
      logo: {
        alt: 'RoboLearn Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Curriculum',
          className: 'curriculum-sidebar-item',
        },
        {
          to: '/dashboard',
          label: 'Dashboard',
          position: 'right',
          className: 'navbar-portal-link',
        },
        // 1. Sign In (Visible on Desktop & Mobile Sidebar)
        {
          to: '/signin',
          label: 'Login',
          position: 'right',
          className: 'navbar-login-link', 
        },
        // 2. Sign Up (Desktop Only - Stylish Button)
        {
          to: '/signup',
          label: 'Sign Up',
          position: 'right',
          className: 'button button--primary navbar-cta desktop-only-item',
        },
        // 3. Sign Up (Mobile Sidebar Only - Aligned Item)
        {
          to: '/signup',
          label: 'Sign Up', 
          position: 'right',
          className: 'mobile-sidebar-cta',
        },
        // 4. Get Started (Mobile Navbar Only - Small Gradient Button)
        {
          to: '/signin',
          label: 'Get Started',
          position: 'right',
          className: 'mobile-navbar-cta',
        },
        {
          label: 'Logout',
          position: 'right',
          to: '#',
          className: 'mobile-only-nav-item logout-sidebar-item',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learning',
          items: [
            {
              label: 'Textbook',
              to: '/docs/part1/foundations-physical-ai',
            },
            {
              label: 'Dashboard',
              to: '/dashboard',
            },
          ],
        },
        {
          title: 'Platform',
          items: [
            {
              label: 'About RoboLearn',
              to: '/',
            },
            {
              label: 'Community',
              href: 'https://discord.com',
            },
            {
              label: 'Support',
              href: 'https://x.com',
            },
          ],
        },
        {
          title: 'Legal',
          items: [
            {
              label: 'Privacy Policy',
              to: '/',
            },
            {
              label: 'Terms of Service',
              to: '/',
            },
          ],
        },
      ],
      copyright: `© ${new Date().getFullYear()} RoboLearn. Built for the Next Generation of Robotics.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
